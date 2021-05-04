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
// KITTI dataset. The laser probe id is determined by  using the vertical
// measurement angle.
struct SweepUncalibrator
{
  int vertId_;
  std::array<std::size_t, 64> probeOrder_;
  SweepUncalibrator() :
    vertId_(0),
    probeOrder_(determine_probe_order(kitti_probe_calibration()))
  {}

  auto get_probeId(const Eigen::Vector3d& point)
  {
    while(vertical_angle_difference(point, kitti_probe_calibration().at(probeOrder_.at(vertId_))) > .002)
    {
      ++vertId_;
      assert(vertId_ < 64);
    }
    return probeOrder_.at(vertId_);
  }
  auto operator()(const Eigen::Vector3d& point)
  {
    auto probeId = get_probeId(point);
    auto [position, distanceUncor] = unapply_calibration(point, kitti_probe_calibration().at(probeId));
    return std::make_tuple(probeId, position, distanceUncor, vertId_);
  }
};

