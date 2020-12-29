#include<iostream>
#include<fstream>
#include<array>
#include<Eigen/Core>

#include<calibration_data.hpp>

struct Probe
{
  double distance;
  double intensity;
};
struct Scan
{
  double position;
  std::array<Probe, 64> probes;
};


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

auto measurementToPointScan(const int laserNum, const Scan& scan, const Calibration& calib)
{
  auto nr_of_probes = 64;
  auto probeId = laserNum % nr_of_probes;
  const auto& cal = calib.at(probeId);
  return measurementToPoint(scan.position, scan.probes.at(probeId).distance, calib.at(probeId));
}

int main(/*int argc, char* argv[]*/)
{
  //std::cout << measurementToPoint(2.1, 4.3, example_probe_calibration()).transpose() << std::endl;
  assert((measurementToPoint(2.1, 4.3, example_probe_calibration()) - Eigen::Vector3d(4.59269, -2.57884, -1.00061)).norm() < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(3.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).first - 3.8 ) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(2.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).first - 2.8 ) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(2.8 , 1.3 , example_probe_calibration()), example_probe_calibration()).first - 2.8 ) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(0.8 , 19.3, example_probe_calibration()), example_probe_calibration()).first - 0.8 ) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(3.14, 19.3, example_probe_calibration()), example_probe_calibration()).first - 3.14) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(3.15, 19.3, example_probe_calibration()), example_probe_calibration()).first - 3.15) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(6.15, 19.3, example_probe_calibration()), example_probe_calibration()).first - 6.15) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(3.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).second - 4.3 ) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(2.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).second - 4.3 ) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(2.8 , 1.3 , example_probe_calibration()), example_probe_calibration()).second - 1.3 ) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(0.8 , 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(3.14, 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(3.15, 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(6.15, 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  return 0;
}

