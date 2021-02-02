#pragma once

#include<Eigen/Core>
#include<calibration_common.hpp>

// An iterative accurate algorithm to unapply the velodyne calibration, i.e.
// compute back the distance measurement and rotational position from a 3d
// point.
// Does not consider some scaling final steps, nor the reflectivity value
auto unapply_calibration_iter(const Eigen::Vector3d point, const ProbeCalibration& cal)
{
  // initial estimate ignores most of the calibration
  auto rotCorrected = atan2(point(0), point(1));
  // iteratively update the position estimate
  for(int i = 10; i--;) // ten fingers...
  {
    auto sinNewRotCorrected = point(0) + cal.horizOffsetCorrection * cos(rotCorrected);
    auto cosNewRotCorrected = point(1) - cal.horizOffsetCorrection * sin(rotCorrected);
    rotCorrected = atan2(sinNewRotCorrected, cosNewRotCorrected);
  }

  auto cosVertAngle = cos(cal.vertCorrection);
  auto distanceUncor = 0.;
  if(fabs(point(0)) > fabs(point(1))) // picking the dimension with the largest value for accuracy
    distanceUncor = (point(0) + cal.horizOffsetCorrection * cos(rotCorrected)) / (cosVertAngle * sin(rotCorrected)) - cal.distCorrection;
  else
    distanceUncor = (point(1) - cal.horizOffsetCorrection * sin(rotCorrected)) / (cosVertAngle * cos(rotCorrected)) - cal.distCorrection;

  auto position = rotCorrected + cal.rotCorrection;

  return std::pair{position, distanceUncor};
}
