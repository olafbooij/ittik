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

  const auto sweep = readSweep(sweepFile);


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
    //static int it = 0; ++it;
    double stepangle = 2 * M_PI / 4000;
    for(auto [probeId, lasers]: enumerate(sweep))
    {
      for(auto laserIt = lasersAtPoseStart.at(probeId); laserIt != lasersAtPoseEnd.at(probeId); ++laserIt)
      {
        auto hori_angle = atan2((*laserIt)(1), (*laserIt)(0));
        auto estimate = liespline::expse3(hori_angle / .05 * liespline::logse3(laserPcloud));
        Eigen::Vector3d pointLaser = estimate * (*laserIt);
        auto [position, distanceUncor] = unapply_calibration(pointLaser, kitti_probe_calibration().at(probeId));
        Eigen::Vector2d sample_error{position - std::lround(position / stepangle) * stepangle,
                                     vertical_angle_difference(pointLaser, kitti_probe_calibration().at(probeId))};
        //std::cout << it << " " << position << " " << probeId << " " << std::lround(position / stepangle) * stepangle << " " << sample_error(1) << std::endl;
        //if(fabs(sample_error(1)) < .005)
        {
          //error(0) += sample_error.norm();
          error(0) += sample_error(0) * sample_error(0);
          //error(1) += sample_error(1) * sample_error(1);
          //std::cout << "e " << sample_error(1) << std::endl;
        }
      }
    }
    error(0) = sqrt(error(0));
    //error(1) = sqrt(error(1));
    //error(0) *= 1000;
    //error(1) *= 1000;
    return error;
  };

  //auto estimate = liespline::Isometryd3::Identity();
  //Eigen::Matrix<double, 6, 1> pose_from_oxts; pose_from_oxts << 0.0101147, -7.00172e-06, 4.35662e-05, 9.10794e-06, 1.32077e-05, 1.83678e-06;
  Eigen::Matrix<double, 6, 1> guess; guess << 0., 0., 0., 0., 0., 0.;
  auto estimate = liespline::expse3(guess);

  {
    double stepangle = 2 * M_PI / 4000;
    for(auto [probeId, lasers]: enumerate(sweep))
      for(auto laserIt = lasersAtPoseStart.at(probeId); laserIt != lasersAtPoseEnd.at(probeId); ++laserIt)
      {
        auto hori_angle = atan2((*laserIt)(1), (*laserIt)(0));
        auto estimate_ = liespline::expse3(hori_angle / .05 * liespline::logse3(estimate));

        Eigen::Vector3d pointLaser = estimate_ * (*laserIt);
        auto [position, distanceUncor] = unapply_calibration(pointLaser, kitti_probe_calibration().at(probeId));
        Eigen::Vector2d sample_error{position - std::lround(position / stepangle) * stepangle,
                                     vertical_angle_difference(pointLaser, kitti_probe_calibration().at(probeId))};
        static std::ofstream file("err");
        file << " " << position << " " << probeId << " " << std::lround(position / stepangle) * stepangle << " " << sample_error(1) << std::endl;
      }
  }
  //errorFunc(estimate);
  for(int i=1e5;i--;)
  //for(int i=1;i--;)
  {
    std::cout << liespline::logse3(estimate).transpose() << " "
              << errorFunc(estimate).transpose() << std::endl;
    estimate = gradient_descent_step(estimate, errorFunc);
  }

  {
    std::array<decltype(sweep.front().begin()), 64> lasersAtPoseStart;
    std::array<decltype(sweep.front().begin()), 64> lasersAtPoseEnd;
    for(auto [probeId, lasers]: enumerate(sweep))
      lasersAtPoseEnd.at(probeId) = lasersAtPoseStart.at(probeId) = lasers.begin();

    for(auto [probeId, lasers]: enumerate(sweep))
    {
      auto& laserEnd = lasersAtPoseEnd.at(probeId);
      while(atan2((*laserEnd)(1), (*laserEnd)(0)) < .2)
        ++laserEnd;
      //std::cout << laserEnd - lasersAtPoseStart.at(probeId) << std::endl;
    }
    double stepangle = 2 * M_PI / 4000;
    for(auto [probeId, lasers]: enumerate(sweep))
      for(auto laserIt = lasersAtPoseStart.at(probeId); laserIt != lasersAtPoseEnd.at(probeId); ++laserIt)
      {
        auto hori_angle = atan2((*laserIt)(1), (*laserIt)(0));
        auto estimate_ = liespline::expse3(hori_angle / .05 * liespline::logse3(estimate));
        Eigen::Vector3d pointLaser = estimate_ * (*laserIt);
        auto [position, distanceUncor] = unapply_calibration(pointLaser, kitti_probe_calibration().at(probeId));
        Eigen::Vector2d sample_error{position - std::lround(position / stepangle) * stepangle,
                                     vertical_angle_difference(pointLaser, kitti_probe_calibration().at(probeId))};
        static std::ofstream file("err_after");
        file << " " << position << " " << probeId << " " << std::lround(position / stepangle) * stepangle << " " << sample_error(1) << std::endl;
      }
  }


  // use some silly generic function to minimize the error by changing the pose...


  return 0;
}

