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
  //std::ofstream outFile(argv[4]);

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

  // take "middle"
  auto point_at_ref = std::lower_bound(points.begin(), points.end(), 0., [](auto a, auto b){return std::get<2>(a) < b;});

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

  // take points until error threshold
  auto estimate = liespline::Isometryd3::Identity();
  double hori_error = 0.;
  auto last_point = point_at_ref;
  while(point_error_func(*last_point, estimate) < stepangle / 4)
    ++last_point;

  std::cout << std::distance(point_at_ref, last_point) << std::endl;

  // unapply and check if they match with a ground truth measurement
  //auto errorFunc = [&](auto laserPcloud, std::string logfilename = std::string())
  auto errorFunc = [&point_at_ref, &point_error_func, &last_point](auto laserPcloud)
  {
    Eigen::Vector2d error{0., 0.};
    for(auto pointIt = point_at_ref; pointIt != last_point; ++pointIt)
    {
      auto hori_error = point_error_func(*pointIt, laserPcloud);
      error(0) += hori_error * hori_error;
      //if(! logfilename.empty())
      //{
      //  static std::ofstream logfile(logfilename); // logging
      //  logfile << " " << position << " " << probeId << " " << std::lround(position / stepangle) * stepangle << " " << sample_error(1) << std::endl;
      //}
    }
    error(0) = sqrt(error(0));
    return error;
  };


  errorFunc(estimate); // logging

  for(int i=1e4;i--;)
  {
    std::cout << liespline::logse3(estimate).transpose() << " "
              << errorFunc(estimate)(0)
              << std::distance(point_at_ref, last_point) << std::endl;
    estimate = gradient_descent_step(estimate, errorFunc);
    while(point_error_func(*last_point, estimate) < stepangle / 4)
      ++last_point;
  }

  errorFunc(estimate); // logging


  return 0;
}

