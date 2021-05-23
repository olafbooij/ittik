#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>

#include"io.hpp"
#include"util.hpp"
#include<unapply_calibration_sweep.hpp>
#include<unapply_calibration_sweep_Triess.hpp>

#include"liespline/se3_plot.hpp"

#include"optimize_lidar_pose.hpp"

auto readSweep(auto&& file)
{
  SweepUncalibratorTriess sweepUncalibrator; // only using the reading part
  std::array<std::vector<Eigen::Vector3d>, 64> sweep;
  Eigen::Vector3d point;
  double refl;
  while(file >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto probeId = sweepUncalibrator.get_probeId(point);
    assert(probeId < 65);
    sweep.at(probeId).emplace_back(point);
  }
  for(const auto& lasers: sweep) assert(! lasers.empty());
  return sweep;
}


int main(int argc, char* argv[])
{
  using namespace ittik;
  std::ifstream sweepFile(argv[1]);
  std::ofstream outFile(argv[2]);

  std::vector<std::tuple<Eigen::Vector3d, int, double>> points;
  {
    const auto sweep = readSweep(sweepFile);

    // flatten
    for(auto [probeId, lasers]: enumerate(sweep))
      for(auto point: lasers)
        points.emplace_back(point, probeId, atan2(point(1), point(0)));
  }

  // sort based on horizontal angle
  std::sort(points.begin(), points.end(), [](auto a, auto b){return std::get<2>(a) < std::get<2>(b);});

  double stepangle = 2 * M_PI / 4000;

  auto point_error_func = [&stepangle](auto point, auto laserPcloud)
  {
    auto& [point_cloud, probeId, hori_angle] = point;
    auto estimate = liespline::expse3(hori_angle / .05 * liespline::logse3(laserPcloud));
    Eigen::Vector3d point_lidar = estimate * point_cloud;
    auto [position, distanceUncor] = unapply_calibration(point_lidar, kitti_probe_calibration().at(probeId));
    auto hori_error = position - std::lround(position / stepangle) * stepangle;
    return hori_error;
  };

  // take "middle"
  auto point_at_ref = std::lower_bound(points.begin(), points.end(), 0., [](auto a, auto b){return std::get<2>(a) < b;});
  auto first_point = point_at_ref;
  auto last_point  = point_at_ref;

  // unapply and check if they match with a ground truth measurement
  auto error_func = [&first_point, &point_error_func, &last_point](auto laserPcloud)
  {
    Eigen::Vector2d error{0., 0.};
    for(auto pointIt = first_point; pointIt != last_point; ++pointIt)
    {
      auto hori_error = point_error_func(*pointIt, laserPcloud);
      error(0) += hori_error * hori_error;
    }
    error(0) = sqrt(error(0) / (last_point - first_point));
    return error;
  };

  // take points until error threshold
  Eigen::Matrix<double, 6, 1> estimate_log; estimate_log << 0.0101147, -7.00172e-06, 4.35662e-05, 9.10794e-06, 1.32077e-05, 1.83678e-06;
  //Eigen::Matrix<double, 6, 1> estimate_log; estimate_log << 0.010, 0., 0., 0., 0., 0.;
  auto estimate = liespline::expse3(estimate_log);
  //auto estimate = liespline::Isometryd3::Identity();

  for(int i=1e4;i--;)
  {
    // add points
    while(point_error_func(*last_point, estimate) < stepangle / 3)
      ++last_point; // .. check if exists...
    // remove points from start
    while(std::get<2>(*first_point) + .1 < std::get<2>(*last_point))
    {
      auto& [point_cloud, probeId, hori_angle] = *first_point;
      auto estimate_ = liespline::expse3(hori_angle / .05 * liespline::logse3(estimate));
      Eigen::Vector3d point_lidar = estimate_ * point_cloud;
      auto [position, distanceUncor] = unapply_calibration(point_lidar, kitti_probe_calibration().at(probeId));
      outFile << probeId << " " << position << " " << distanceUncor << std::endl;
      ++first_point;
    }

    std::cout << liespline::logse3(estimate).transpose() << " "
              << error_func(estimate)(0) << " "
              << first_point - point_at_ref << " "
              << last_point - first_point << std::endl;
    estimate = gradient_descent_step(estimate, error_func);
  }

  return 0;
}

