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
  auto position = atan2(point(0), point(1)) + atan2(cal.horizOffsetCorrection, sqrt(distXYSquared)) + cal.rotCorrection;
  return std::pair{position, distanceUncor};
}

auto unapply_calibration_unknown_laser(const Eigen::Vector3d point, const Calibration& cals)
{
  auto distXYSquared = point(0) * point(0) + point(1) * point(1);
  // pick closest calibration
  auto calp = (std::min_element(cals.begin(), cals.end(), [&point, &distXYSquared](auto calA, auto calB){
    auto vertCorrectionA = atan((point(2) - calA.vertOffsetCorrection) / sqrt(distXYSquared));
    auto vertCorrectionB = atan((point(2) - calB.vertOffsetCorrection) / sqrt(distXYSquared));
    return fabs(calA.vertCorrection - vertCorrectionA) < fabs(calB.vertCorrection - vertCorrectionB);}));
  auto cal = *calp;
  // should check if the result is significant. If not, then it's probably some
  // sort of spurious measurement, which does not have a specific semantic label
  //assert(fabs(cal.vertCorrection - vertCorrection) <
  auto distanceUncor = sqrt(distXYSquared - cal.horizOffsetCorrection * cal.horizOffsetCorrection) / cos(cal.vertCorrection) - cal.distCorrection;
  auto position = atan2(point(0), point(1)) + atan2(cal.horizOffsetCorrection, sqrt(distXYSquared)) + cal.rotCorrection;
  return std::tuple{position, distanceUncor, calp - cals.begin()};
}
