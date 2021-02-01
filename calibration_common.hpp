#pragma once

#include<array>
#include<cmath>

struct ProbeCalibration
{
  double rotCorrection; // radians
  double vertCorrection; // radians
  double distCorrection; // m
  double distCorrectionX; // m
  double distCorrectionY; // m
  double vertOffsetCorrection; // m
  double horizOffsetCorrection; // m
};

using Calibration = std::array<ProbeCalibration, 64>;

namespace nonsi {

  struct _ProbeCalibration // strange units
  {
    double rotCorrection; // degrees
    double vertCorrection; // degrees
    double distCorrection; // cm
    double distCorrectionX; // cm
    double distCorrectionY; // cm
    double vertOffsetCorrection; // cm
    double horizOffsetCorrection; // cm
  };
  using _Calibration = std::array<_ProbeCalibration, 64>;

  auto example__probe_calibration()
  {
    return _ProbeCalibration{1.0275339 , -12.115005 , 107.46816 , 110.37566 , 104.23552 , 12.081173 , -2.5999999};
  }

  auto common_units(const _ProbeCalibration _probeCalibration)
  {
    return ProbeCalibration{
      _probeCalibration.rotCorrection         / 180 * M_PI,
      _probeCalibration.vertCorrection        / 180 * M_PI,
      _probeCalibration.distCorrection        / 100,
      _probeCalibration.distCorrection        / 100,
      _probeCalibration.distCorrection        / 100,
      _probeCalibration.vertOffsetCorrection  / 100,
      _probeCalibration.horizOffsetCorrection / 100
    };
  }

  auto common_units(const _Calibration _calibration)
  {
    Calibration calibration;
    for(int i = 0; i < _calibration.size(); ++i)
      calibration.at(i) = common_units(_calibration.at(i));
    return calibration;
  }
}

auto example_probe_calibration()
{
  return nonsi::common_units(nonsi::example__probe_calibration());
}
