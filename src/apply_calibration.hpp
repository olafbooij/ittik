#pragma once

#include<Eigen/Core>
#include<calibration_common.hpp>

// Part of the algorithm taken from the Coordinate Calculation Algorithm Sample
// Code from Appendix F of the Velodyne HDL-64E S2 and S2.1 User's manual and
// programming guide, revision 63HDL64E S2 Rev F DEC11
// But without the x and y dependent distance correction.

Eigen::Vector3d apply_calibration(const double position, const double distanceUncor, const ProbeCalibration& cal)
{
  if(distanceUncor == 0) // error value
    return {.0, .0, .0};
  auto distance = distanceUncor + cal.distCorrection;
  auto rotCorrected = position + cal.rotCorrection;
  double cosRotAngle = cos(rotCorrected);
  double sinRotAngle = sin(rotCorrected);

  auto cosVertAngle = cos(cal.vertCorrection);
  auto sinVertAngle = sin(cal.vertCorrection);
  double xyDistance = distance * cosVertAngle;

  Eigen::Vector3d point;
  point(0) = xyDistance * cosRotAngle - cal.horizOffsetCorrection * sinRotAngle;
  point(1) = xyDistance * sinRotAngle + cal.horizOffsetCorrection * cosRotAngle;
  point(2) = distance * sinVertAngle + cal.vertOffsetCorrection;

  return point;
}

