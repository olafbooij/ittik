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
  auto position = rotCorrected + cal.rotCorrection;
  auto distance = point.norm();
  auto distanceUncor = distance - cal.distCorrection;
  // iteratively update the position and distanceUncor estimate
  for(int i = 10; i--;) // ten fingers...
  {
    // mostly copy-paste from forward apply_calibration
    auto distance = distanceUncor + cal.distCorrection;
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

    auto distanceXCorrected = distanceUncor + distanceCorrX;
    auto xyDistanceXCorrected = distanceXCorrected * cosVertAngle;
    auto distanceYCorrected = distanceUncor + distanceCorrY;
    auto xyDistanceYCorrected = distanceYCorrected * cosVertAngle;

    // use the point to compute a more accurate position and distance
    auto sinNewRotCorrected = (point(0) + cal.horizOffsetCorrection * cosRotAngle) / xyDistanceXCorrected;
    auto cosNewRotCorrected = (point(1) - cal.horizOffsetCorrection * sinRotAngle) / xyDistanceYCorrected;
    rotCorrected = atan2(sinNewRotCorrected, cosNewRotCorrected);

    if(fabs(point(0)) > fabs(point(1))) // picking the dimension with the largest value for accuracy
      distanceUncor = (point(0) + cal.horizOffsetCorrection * cosRotAngle) / (cosVertAngle * sinRotAngle) - distanceCorrX;
    else
      distanceUncor = (point(1) - cal.horizOffsetCorrection * sinRotAngle) / (cosVertAngle * cosRotAngle) - distanceCorrY;

    //distanceUncor = (point(2) - cal.vertOffsetCorrection) / sinVertAngle - distanceCorrY;

    position = rotCorrected + cal.rotCorrection;
  }

  return std::pair{position, distanceUncor};
}
