#include<fstream>
#include<Eigen/Core>

#include<unapply_calibration_sweep.hpp>


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
    outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << std::endl;
  }

  return 0;
}

