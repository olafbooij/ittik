#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>

#include"io.hpp"
#include"util.hpp"
#include"unapply_calibration_sweep.hpp"
#include"unapply_calibration_sweep_Triess.hpp"

#include"liespline/se3_plot.hpp"

#include"optimize_lidar_pose_euclid.hpp"


struct Point
{
  Eigen::Vector3d point;
  std::size_t     probeId;
  double          hori_angle;
  double          position;
  double          distanceUncor;
};

auto readSweep(auto&& file)
{
  SweepUncalibratorTriess sweepUncalibrator; // only using the reading part
  std::vector<Point> points;
  Eigen::Vector3d point;
  double refl;
  while(file >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
    assert(probeId < 65);
    points.emplace_back(Point{point, probeId, atan2(point(1), point(0)), position, distanceUncor});
  }
  return points;
}


int main(int argc, char* argv[])
{
  using namespace ittik;
  std::ifstream raw_sweep_file(argv[1]);
  std::ifstream corrected_sweep_file(argv[2]);
  //std::ofstream outFile(argv[3]);

  const auto raw_sweep = readSweep(raw_sweep_file);
  assert(! raw_sweep.empty());
  const auto corrected_sweep = readSweep(corrected_sweep_file);
  assert(corrected_sweep.size() == raw_sweep.size());

  // define error between corresponding points given pose-spline (don't even have to bother with calibration)
  // iteratively estimate
  auto point_error_func = [](auto raw_point, auto corrected_point, auto laserPcloud)
  {
    auto estimate = liespline::expse3(raw_point.hori_angle * laserPcloud);
    Eigen::Vector3d corrected_point_lidar = estimate * corrected_point.point;
    //Eigen::Vector3d error = corrected_point_lidar - raw_point.point;
    Eigen::Matrix<double, 1, 1> error; error(0) = acos(std::clamp(corrected_point_lidar.normalized().dot(raw_point.point.normalized()), -1., 1.));
    return error;
  };

  auto error_func = [&raw_sweep, &corrected_sweep, &point_error_func](auto laserPcloud)
  {
    Eigen::Vector2d error{0., 0.};
    int n = 0;
    for(auto point_i = 0; point_i < raw_sweep.size(); ++point_i)
      if(point_i % 10 == 0)
    {
      auto point_error = point_error_func(raw_sweep.at(point_i), corrected_sweep.at(point_i), laserPcloud);
      error(0) += point_error.squaredNorm();
      ++n;
    }
    error(0) = sqrt(error(0) / n);
    return error;
  };

  // error based on local angle difference
  // get 3 points in a row. if approx the same distance and same
  // position-delta, take difference of position-delta and standard-distance as
  // error. (...could results in outliers...)
  auto delta_error_func = [&corrected_sweep](auto laserPcloud)
  {
    double prev_position = 0;
    double prev_distanceUncor = 0;
    double prev_delta = 0;
    auto prev_point = corrected_sweep.front();
    int okay = 0;
    Eigen::Vector2d error{0., 0.};
    int n = 0;
    for(auto point: corrected_sweep)
    {
      // determine correspondences based on original cloud
      auto [position, distanceUncor] = unapply_calibration(point.point, kitti_probe_calibration().at(point.probeId));
      auto delta = position - prev_position;
      if((fabs(distanceUncor - prev_distanceUncor) > .05) ||
         (fabs(delta - prev_delta) > .001)) // bit of a though one, should maybe make it smaller at the cost of inliers
        okay = 0;
      else
        ++okay;
      prev_position = position;
      prev_distanceUncor = distanceUncor;
      prev_delta = delta;

      if(okay > 1)
      {
        // get updated cloud
        auto get_updated_position = [&laserPcloud](auto& point)
        {
          auto estimate0 = liespline::expse3(point.hori_angle * laserPcloud);
          Eigen::Vector3d point_lidar0 = estimate0 * point.point;
          auto estimate = liespline::expse3(atan2(point_lidar0(1), point_lidar0(0)) * laserPcloud);
          Eigen::Vector3d point_lidar = estimate * point.point;
          auto [position, _distanceUncor] = unapply_calibration(point_lidar, kitti_probe_calibration().at(point.probeId));
          return position;
        };

        auto position_up = get_updated_position(point);
        auto prev_position_up = get_updated_position(prev_point);

        ++n;
        error(0) += pow((position_up - prev_position_up) - M_PI / 1000, 2);
      }
      prev_point = point;
    }
    //std::cout << n << std::endl;
    if(n)
      error(0) = sqrt(error(0) / n);
    return error;
  };

  auto vert_error_func = [&corrected_sweep](auto laserPcloud)
  {
    Eigen::Vector2d error{0., 0.};
    for(auto point: corrected_sweep)
    {
      auto [position, distanceUncor] = unapply_calibration(point.point, kitti_probe_calibration().at(point.probeId));
      if(fabs(position) < .05) // there might be probe id error around position 0
      {
        auto estimate0 = liespline::expse3(point.hori_angle * laserPcloud);
        Eigen::Vector3d point_lidar0 = estimate0 * point.point;
        auto estimate = liespline::expse3(atan2(point_lidar0(1), point_lidar0(0)) * laserPcloud);
        Eigen::Vector3d point_lidar = estimate * point.point;
        auto vert_error = vertical_angle_difference(point_lidar, kitti_probe_calibration().at(point.probeId));
        error(0) += vert_error * vert_error;
      }
    }
    error(0) = sqrt(error(0));
    return error;
  };

  //auto write_sweeps = [&raw_sweep, &corrected_sweep, &point_error_func](auto laserPcloud)
  //{
  //  std::ofstream file("raw_corrected");
  //  for(int point_i = 0; point_i < raw_sweep.size(); ++point_i)
  //    //if(point_i % 100 == 0)
  //      //file << point_error_func(raw_sweep.at(point_i), corrected_sweep.at(point_i), laserPcloud).transpose() << std::endl;
  //      file << (point_error_func(raw_sweep.at(point_i), corrected_sweep.at(point_i), laserPcloud) + raw_sweep.at(point_i).point).transpose() << " 0" << std::endl;
  //};

  //Eigen::Matrix<double, 6, 1> estimate; estimate << 0.200362, -0.000649443,  0.00181078, 0.000137006, 0.000205667, 3.21881e-05;
  Eigen::Matrix<double, 6, 1> estimate; estimate << 0., 0., 0., 0., 0., 0.;
  //write_sweeps(estimate);
  for(int i=1e3;i--;)
  {
    //if(i%20 == 0)
    std::cout //<< delta_error_func(estimate)(0) << " "
              << error_func(estimate)(0) << " "
              << vert_error_func(estimate)(0) << " "
              << estimate.transpose() << std::endl;
    estimate = gradient_descent_step(estimate, error_func);
    //estimate = gradient_descent_step(estimate, delta_error_func);
  }

  return 0;
}

