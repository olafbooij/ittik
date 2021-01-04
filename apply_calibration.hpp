#pragma once

#include<Eigen/Core>
#include<calibration_common.hpp>

// Algorithm taken from the Coordinate Calculation Algorithm Sample Code from
// Appendix F of the Velodyne HDL-64E S2 and S2.1 User's manual and programming
// guide, revision 63HDL64E S2 Rev F DEC11

Eigen::Vector3d apply_calibration(const double position, const double distanceUncor, const ProbeCalibration& cal)
{
  if(distanceUncor == 0) // error value
    return {.0, .0, .0};
  auto distance = distanceUncor + cal.distCorrection;
  auto rotCorrected = position - cal.rotCorrection;
  double cosRotAngle = cos(rotCorrected);
  double sinRotAngle = sin(rotCorrected);

  auto cosVertAngle = cos(cal.vertCorrection);
  auto sinVertAngle = sin(cal.vertCorrection);
  double xyDistance = distance * cosVertAngle;
  // Asuming calibration target was relative to lidar.
  double xx = fabs(xyDistance * sinRotAngle - cal.horizOffsetCorrection * cosRotAngle);
  double yy = fabs(xyDistance * cosRotAngle + cal.horizOffsetCorrection * sinRotAngle);

  // These exact values are used to calibrate all units it seems.
  double distanceCorrX = (cal.distCorrection - cal.distCorrectionX) * (xx - 2.40) / (25.04 - 2.40) + cal.distCorrectionX;
  double distanceCorrY = (cal.distCorrection - cal.distCorrectionY) * (yy - 1.93) / (25.04 - 1.93) + cal.distCorrectionY;

  Eigen::Vector3d point;
  auto distance_x_corrected = distanceUncor + distanceCorrX;
  auto xyDistance_x_corrected = distance_x_corrected * cosVertAngle;
  point(0) = xyDistance_x_corrected * sinRotAngle - cal.horizOffsetCorrection * cosRotAngle;

  auto distance_y_corrected = distanceUncor + distanceCorrY;
  auto xyDistance_y_corrected = distance_y_corrected * cosVertAngle;
  point(1) = xyDistance_y_corrected * cosRotAngle + cal.horizOffsetCorrection * sinRotAngle;

  point(2) = distance_y_corrected * sinVertAngle + cal.vertOffsetCorrection; // yes distance_y_corrected is used

  return point;
}

