#pragma once

#include<calibration_common.hpp>
#include<apply_calibration.hpp>

auto pointToMeasurement_iter(const Eigen::Vector3d point, const ProbeCalibration& cal)
{
  auto rotCorrected = atan2(point(0), point(1));
  auto distance = point.norm();
  auto distance_uncor = distance - cal.distCorrection;
  for(int i = 0; i--;)
  {
    double cosRotAngle = cos(rotCorrected);
    double sinRotAngle = sin(rotCorrected);

    auto cosVertAngle = cos(cal.vertCorrection);
    auto sinVertAngle = sin(cal.vertCorrection);
    double xyDistance = distance * cosVertAngle;
    double xx = fabs(xyDistance * sinRotAngle - cal.horizOffsetCorrection * cosRotAngle);
    double yy = fabs(xyDistance * cosRotAngle + cal.horizOffsetCorrection * sinRotAngle);
    double distanceCorrX = (cal.distCorrection - cal.distCorrectionX) * (xx - 2.40) / (25.04 - 2.40) + cal.distCorrectionX;
    double distanceCorrY = (cal.distCorrection - cal.distCorrectionY) * (yy - 1.93) / (25.04 - 1.93) + cal.distCorrectionY;

    auto distance_x_corrected = distance_uncor + distanceCorrX;
    auto xyDistance_x_corrected = distance_x_corrected * cosVertAngle;
    auto distance_y_corrected = distance_uncor + distanceCorrY;
    auto xyDistance_y_corrected = distance_y_corrected * cosVertAngle;

    //auto new_rotCorrected = atan2(point(0) + 
  }
  auto position = rotCorrected + cal.rotCorrection;

  return std::pair{position, distance_uncor};
}
