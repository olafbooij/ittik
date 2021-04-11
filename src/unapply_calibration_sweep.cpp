#include<fstream>
#include<Eigen/Core>

#include<unapply_calibration_sweep.hpp>

// Read in a sweep file from the KITTI dataset and outputs raw measurement data (ignoring reflectivity values).
// example usage:
// ./unapply_calibration_sweep $KITTIDIR/2011_09_26/2011_09_26_drive_0002_extract/velodyne_points/data/0000000071.txt ittiked.sweep > debug
int main(int argc, char* argv[])
{
  SweepUncalibrator sweepUncalibrator;

  std::ifstream sweepFile(argv[1]);
  std::ofstream outFile(argv[2]);
  Eigen::Vector3d point;
  double refl;

  while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
    double onestep = 2 * M_PI / 4000;
    long pix = std::lround(position / onestep) + 1999;
    if(pix == -1) pix = 3999;
    outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << " " << pix << std::endl;
  }
  return 0;
}

