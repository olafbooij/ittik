#pragma once

#include<Eigen/Core>
#include<calibration_common.hpp>

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

auto measurementToPointScan(const int laserNum, const Scan& scan, const Calibration& calib)
{
  auto nr_of_probes = 64;
  auto probeId = laserNum % nr_of_probes;
  const auto& cal = calib.at(probeId);
  return measurementToPoint(scan.position, scan.probes.at(probeId).distance, calib.at(probeId));
}

