#pragma once

#include<Eigen/Core>
#include<calibration_common.hpp>

// An algorithm to unapply the velodyne calibration,
// i.e. compute back the distance measurement and rotational position from a 3d
// point.
// Does not consider some scaling final steps, nor the reflectivity value
auto unapply_calibration(const Eigen::Vector3d point, const ProbeCalibration& cal)
{
  auto distXYSquared = point(0) * point(0) + point(1) * point(1);
  auto distanceUncor = sqrt(distXYSquared - cal.horizOffsetCorrection * cal.horizOffsetCorrection) / cos(cal.vertCorrection) - cal.distCorrection;
  auto position = atan2(point(1), point(0)) - atan2(cal.horizOffsetCorrection, sqrt(distXYSquared)) - cal.rotCorrection;
  if(position < -M_PI)
    position += 2 * M_PI;
  if(position > M_PI)
    position -= 2 * M_PI;
  return std::pair{position, distanceUncor};
}

// Function that can be used to determine if a certain point was measured by a
// certian laser probe.
auto vertical_angle_difference(const Eigen::Vector3d point, const ProbeCalibration& cal)
{
  auto distXYSquared = point(0) * point(0) + point(1) * point(1);
  auto vertCorrection = atan((point(2) - cal.vertOffsetCorrection) / sqrt(distXYSquared));
  return cal.vertCorrection - vertCorrection;
}
