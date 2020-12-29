#pragma once

#include<Eigen/Core>
#include<calibration_common.hpp>

Eigen::Vector3d measurementToPoint(const double position, const double distance_uncor, const ProbeCalibration& cal)
{
  if(distance_uncor == 0) // error value
    return {.0, .0, .0};
  auto distance = distance_uncor + cal.distCorrection;
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
  auto distance_x_corrected = distance_uncor + distanceCorrX;
  auto xyDistance_x_corrected = distance_x_corrected * cosVertAngle;
  point(0) = xyDistance_x_corrected * sinRotAngle - cal.horizOffsetCorrection * cosRotAngle;

  auto distance_y_corrected = distance_uncor + distanceCorrY;

  auto xyDistance_y_corrected = distance_y_corrected * cosVertAngle;
  point(1) = xyDistance_y_corrected * cosRotAngle + cal.horizOffsetCorrection * sinRotAngle;
  point(2) = distance_y_corrected * sinVertAngle + cal.vertOffsetCorrection; // yes distance_y_corrected is used

  return point;
}

