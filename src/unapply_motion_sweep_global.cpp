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
    Eigen::Vector3d error = corrected_point_lidar - raw_point.point;
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
    error(1) = sqrt(error(1) / n);
    return error;
  };

  // error based on local angle difference
  // get 3 points in a row. if approx the same distance and same
  // position-delta, take difference of position-delta and standard-distance as
  // error. (...could results in outliers...)
  //{
  //  auto prev_point = corrected_sweep.begin();
  //  for(auto point: corrected_sweep)
  //  {
  //    
  //  }

  //}







  auto write_sweeps = [&raw_sweep, &corrected_sweep, &point_error_func](auto laserPcloud)
  {
    std::ofstream file("raw_corrected");
    for(int point_i = 0; point_i < raw_sweep.size(); ++point_i)
      //if(point_i % 100 == 0)
        //file << point_error_func(raw_sweep.at(point_i), corrected_sweep.at(point_i), laserPcloud).transpose() << std::endl;
        file << (point_error_func(raw_sweep.at(point_i), corrected_sweep.at(point_i), laserPcloud) + raw_sweep.at(point_i).point).transpose() << " 0" << std::endl;
  };

  //auto estimate = liespline::Isometryd3::Identity();
  Eigen::Matrix<double, 6, 1> estimate; estimate << 0.200362, -0.000649443,  0.00181078, 0.000137006, 0.000205667, 3.21881e-05;
  write_sweeps(estimate);
  for(int i=1e4;i--;)
  {
    estimate = gradient_descent_step(estimate, error_func);
    std::cout << error_func(estimate)(0) << " " << estimate.transpose() << std::endl;
  }

  return 0;
}

