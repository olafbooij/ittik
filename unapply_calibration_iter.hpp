#pragma once

#include<calibration_common.hpp>
#include<apply_calibration.hpp>

auto pointToMeasurement_iter(const Eigen::Vector3d point, const ProbeCalibration& cal)
{
  // initial estimate ignores most of the calibration
  auto rotCorrected = atan2(point(0), point(1));
  auto position = rotCorrected + cal.rotCorrection;
  auto distance = point.norm();
  auto distance_uncor = distance - cal.distCorrection;
  // iteratively update the estimate
  for(int i = 10; i--;)
  {
    // mostly copy=paste from forward measurementToPoint
    auto distance = distance_uncor + cal.distCorrection;
    auto rotCorrected = position - cal.rotCorrection;
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

    // use the point to compute a more accurate position and distance
    auto sin_new_rotCorrected = (point(0) + cal.horizOffsetCorrection * cosRotAngle) / xyDistance_x_corrected;
    auto cos_new_rotCorrected = (point(1) - cal.horizOffsetCorrection * sinRotAngle) / xyDistance_y_corrected;
    rotCorrected = atan2(sin_new_rotCorrected, cos_new_rotCorrected);

    if(fabs(point(0)) > fabs(point(1)))
      distance_uncor = (point(0) + cal.horizOffsetCorrection * cosRotAngle) / (cosVertAngle * sinRotAngle) - distanceCorrX;
    else
      distance_uncor = (point(1) - cal.horizOffsetCorrection * sinRotAngle) / (cosVertAngle * cosRotAngle) - distanceCorrY;

    //distance_uncor = (point(2) - cal.vertOffsetCorrection) / sinVertAngle - distanceCorrY;

    position = rotCorrected + cal.rotCorrection;
  }

  return std::pair{position, distance_uncor};
}
