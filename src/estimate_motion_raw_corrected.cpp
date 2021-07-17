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

auto readSweep(auto&& file)
{
  SweepUncalibratorTriess sweepUncalibrator; // only using the reading part
  std::vector<std::tuple<Eigen::Vector3d, int, double>> points;
  Eigen::Vector3d point;
  double refl;
  while(file >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto probeId = sweepUncalibrator.get_probeId(point);
    assert(probeId < 65);
    points.emplace_back(point, probeId, atan2(point(1), point(0)));
  }
  return points;
}


int main(int argc, char* argv[])
{
  using namespace ittik;
  std::ifstream raw_sweep_file(argv[1]);
  std::ifstream corrected_sweep_file(argv[2]);
  //std::ofstream outFile(argv[3]);

  // read raw data
  const auto raw_sweep = readSweep(raw_sweep_file);
  // read corrected data
  const auto corrected_sweep = readSweep(corrected_sweep_file);
  // define error between corresponding points given pose-spline (don't even have to bother with calibration)
  // iteratively estimate
  auto point_error_func = [](auto raw_point, auto corrected_point, auto laserPcloud)
  {
    auto& [raw_point_cloud, raw_probeId, raw_hori_angle] = raw_point;
    auto estimate = liespline::expse3(raw_hori_angle * laserPcloud);
    auto& [corrected_point_cloud, corrected_probeId, corrected_hori_angle] = corrected_point;
    Eigen::Vector3d corrected_point_lidar = estimate * corrected_point_cloud;
    Eigen::Vector3d error = corrected_point_lidar - raw_point_cloud;
    return error;
  };
  auto error_func = [&raw_sweep, &corrected_sweep, &point_error_func](auto laserPcloud)
  {
    Eigen::Vector2d error{0., 0.};
    int i = 0 ; //iter;
    int n = 0;
    for(auto pointI = 0; pointI < raw_sweep.size(); ++pointI)
      if(++i % 10 == 0)
    {
      auto point_error = point_error_func(raw_sweep.at(pointI), corrected_sweep.at(pointI), laserPcloud);
      error(0) += point_error.squaredNorm();
      ++n;
    }
    error(0) = sqrt(error(0) / n);
    error(1) = sqrt(error(1) / n);
    std::cout << error(0) << " " << laserPcloud.transpose() << std::endl;
    return error;
  };

  Eigen::Matrix<double, 6, 1> estimate; estimate << 0.2, 0., 0., 0., 0., 0.; // per radians (.20m*2*pi*10*3.6 = 45km/h)
  //auto estimate = liespline::Isometryd3::Identity();
  for(int i=1e4;i--;)
    estimate = gradient_descent_step(estimate, error_func);

  return 0;
}

