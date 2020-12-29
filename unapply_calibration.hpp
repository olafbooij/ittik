#pragma once

#include<Eigen/Core>
#include<calibration_common.hpp>

auto pointToMeasurement(const Eigen::Vector3d point, const ProbeCalibration& cal)
{
  auto cosVertAngle = cos(cal.vertCorrection);
  auto sinVertAngle = sin(cal.vertCorrection);
  auto distance_y_corrected = (point(2) - cal.vertOffsetCorrection) / sinVertAngle;
  auto xyDistance_y_corrected = distance_y_corrected * cosVertAngle;

  double position;
  double rotCorrected;
  {
    auto a = xyDistance_y_corrected;
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
  double distance_uncor;
  {
    auto a = (cal.distCorrection - cal.distCorrectionY)  / (25.04 - 1.93);
    auto b = a * cosVertAngle * cosRotAngle;

    auto distance_uncor_ = (distance_y_corrected - cal.distCorrectionY  + b * cal.distCorrection + a * (cal.horizOffsetCorrection * sinRotAngle + 1.93)) / (1 - b);

    auto distance_uncor__ = (distance_y_corrected - cal.distCorrectionY - b * cal.distCorrection - a * (cal.horizOffsetCorrection * sinRotAngle - 1.93)) / (1 + b);
    distance_uncor = std::min(distance_uncor_, distance_uncor__);
  }
  return std::pair{position, distance_uncor};
}

