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
  //ifstream sweepFile(argv[1]);
  //ofstream outFile(argv[4]);

  const auto sweep = readSweep(std::ifstream("00_odo.txt"));


  std::array<decltype(sweep.front().begin()), 64> lasersAtPoseStart;
  std::array<decltype(sweep.front().begin()), 64> lasersAtPoseEnd;
  for(auto [probeId, lasers]: enumerate(sweep))
    lasersAtPoseEnd.at(probeId) = lasersAtPoseStart.at(probeId) = lasers.begin();

  for(auto [probeId, lasers]: enumerate(sweep))
  {
    auto& laserEnd = lasersAtPoseEnd.at(probeId);
    while(atan2((*laserEnd)(1), (*laserEnd)(0)) < .1)
      ++laserEnd;
    //std::cout << laserEnd - lasersAtPoseStart.at(probeId) << std::endl;
  }

  // unapply and check if they match with a ground truth measurement
  auto errorFunc = [&](auto laserPcloud)
  {
    Eigen::Vector2d error{0., 0.};
    double stepangle = 2 * M_PI / 4000;
    for(auto [probeId, lasers]: enumerate(sweep))
    {
      for(auto laserIt = lasersAtPoseStart.at(probeId); laserIt != lasersAtPoseEnd.at(probeId); ++laserIt)
      {
        auto pointLaser = laserPcloud * (*laserIt);
        auto [position, distanceUncor] = unapply_calibration(pointLaser, kitti_probe_calibration().at(probeId));
        Eigen::Vector2d sample_error{position - std::lround(position / stepangle) * stepangle,
                                     vertical_angle_difference(pointLaser, kitti_probe_calibration().at(probeId))};
        //error(0) += sample_error.norm();
        error(0) += fabs(sample_error(0));
        //error(1) += fabs(sample_error(1));
      }
    }
    return error;
  };
  std::cout << errorFunc(liespline::Isometryd3::Identity()) << std::endl;
  auto estimate = liespline::Isometryd3::Identity();
  for(int i=9000;i--;)
    std::cout << liespline::logse3(estimate).transpose() << " "
              << errorFunc(estimate = gradient_descent_step(estimate, errorFunc)).transpose()  << std::endl;
              // ...try out gaus-newton... -> levenberg marquardt



  // use some silly generic function to minimize the error by changing the pose...


  return 0;
}

