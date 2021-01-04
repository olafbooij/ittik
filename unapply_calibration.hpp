#pragma once

#include<Eigen/Core>
#include<calibration_common.hpp>

// A direct, but rather unstable algorithm to unapply the velodyne calibration,
// i.e. compute back the distance measurement and rotational position from a 3d
// point.
// Does not consider some scaling final steps, nor the reflectivity value
auto unapply_calibration(const Eigen::Vector3d point, const ProbeCalibration& cal)
{
  auto cosVertAngle = cos(cal.vertCorrection);
  auto sinVertAngle = sin(cal.vertCorrection);
  auto distanceYCorrected = (point(2) - cal.vertOffsetCorrection) / sinVertAngle;
  auto xyDistanceYCorrected = distanceYCorrected * cosVertAngle;

  double position;
  double rotCorrected;
  {
    auto a = xyDistanceYCorrected;
    auto b = cal.horizOffsetCorrection;
    auto c = point(1);
    auto a2 = a * a;
    auto b2 = b * b;
    auto c2 = c * c;
    if(atan2(point(0), point(1)) > 0)
    {
      auto cosRotAngle = (a * c - b * sqrt(a2 + b2 - c2)) / (a2 + b2);
      rotCorrected = acos(cosRotAngle);
      position = rotCorrected + cal.rotCorrection;
    }
    else
    {
      auto cosRotAngle = (a * c + b * sqrt(a2 + b2 - c2)) / (a2 + b2);
      rotCorrected = acos(cosRotAngle);
      position = 2 * M_PI - rotCorrected + cal.rotCorrection;
    }
  }
  auto cosRotAngle = cos(rotCorrected);
  auto sinRotAngle = sin(rotCorrected);
  double distanceUncor;
  {
    auto a = (cal.distCorrection - cal.distCorrectionY) / (25.04 - 1.93);
    auto b = a * cosVertAngle * cosRotAngle;

    auto distanceUncor_ = (distanceYCorrected - cal.distCorrectionY  + b * cal.distCorrection + a * (cal.horizOffsetCorrection * sinRotAngle + 1.93)) / (1 - b);

    auto distanceUncor__ = (distanceYCorrected - cal.distCorrectionY - b * cal.distCorrection - a * (cal.horizOffsetCorrection * sinRotAngle - 1.93)) / (1 + b);
    distanceUncor = std::min(distanceUncor_, distanceUncor__);
  }
  return std::pair{position, distanceUncor};
}

