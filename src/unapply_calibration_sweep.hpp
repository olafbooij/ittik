#pragma once

#include<tuple>
#include<Eigen/Core>
#include<iostream>

#include<calibration_kitti.hpp>
#include<unapply_calibration.hpp>

// Get the probe order based vertical angle.
auto determine_probe_order(const Calibration calibration)
{
  std::array<std::size_t, calibration.size()> perm;
  for(size_t i = 0; i < calibration.size(); ++i) perm.at(i) = i;
  std::sort(perm.begin(), perm.end(), [&calibration](auto a, auto b){return calibration.at(a).vertCorrection > calibration.at(b).vertCorrection;});
  return perm;
}

// This helper class should be fed with points in the order as stored in the
// KITTI dataset. The probe number for each point can then be simply determined
// as described in:
// "Scan-based semantic segmentation of lidar point clouds: An experimental study",
// Triess, L.T., Peter, D., Rist, C.B., Zöllner, J.M., arXiv preprint arXiv:2004.11803 
// Uh.... no... not for some datasets where there's (almost) no measurements
// from the lasers pointing up. Changed it now to using the vertical
// measurement angle to determine the laser id.
struct SweepUncalibrator
{
  int vertId_;
  //double horizontalAnglePrev_;
  std::array<std::size_t, 64> probeOrder_;
  SweepUncalibrator() :
    vertId_(0),
    //horizontalAnglePrev_(10),
    probeOrder_(determine_probe_order(kitti_probe_calibration()))
  {}

  auto read(const Eigen::Vector3d& point)
  {
    //double horizontalAngle = atan2(point(1), point(0));
    //if(horizontalAngle < 0)
    //  horizontalAngle += 2 * M_PI;
    //float small_angle = .1;
    //if(horizontalAngle + small_angle < horizontalAnglePrev_)  // the + small_angle is just for rebustness needed for motion corrected data
      while((vertical_angle_difference(point, kitti_probe_calibration().at(probeOrder_.at(vertId_)))) > .002)
      {
        ++vertId_;
        assert(vertId_ < 64);
      }
    //print_vertical_angle_difference(point, kitti_probe_calibration().at(probeOrder_.at(vertId_)));
    //horizontalAnglePrev_ = horizontalAngle;
    return probeOrder_.at(vertId_);
  }

  auto operator()(const Eigen::Vector3d& point)
  {
    auto probeId = read(point);
    auto [position, distanceUncor] = unapply_calibration(point, kitti_probe_calibration().at(probeId));
    return std::make_tuple(probeId, position, distanceUncor, vertId_);
  }
};

