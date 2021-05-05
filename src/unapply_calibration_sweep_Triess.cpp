#include<fstream>
#include<Eigen/Core>
#undef NDEBUG
#include <cassert>
#define NDEBUG

#include"unapply_calibration_sweep_Triess.hpp"

// Read in a sweep file from the KITTI dataset and outputs raw measurement data (ignoring reflectivity values).
// example usage:
// ./unapply_calibration_sweep $KITTIDIR/2011_09_26/2011_09_26_drive_0002_extract/velodyne_points/data/0000000071.txt ittiked.sweep
int main(int argc, char* argv[])
{
  SweepUncalibratorTriess sweepUncalibrator;

  std::ifstream sweepFile(argv[1]);
  std::ofstream outFile(argv[2]);
  auto calibration = kitti_probe_calibration();

  Eigen::Vector3d point;
  double refl;
  while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
    outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << " " << std::endl;
  }
  assert(sweepUncalibrator.vertId_ == 63);

  return 0;
}

