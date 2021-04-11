#include<fstream>
#include<Eigen/Core>

#include<unapply_calibration_sweep.hpp>

// The points in the kitti point clouds are taken from a single 2 pi sweep of
// the lidar sensor. Because of the different horizontal direction of the
// individual lasers this results in a sweep of more than 2 pi.
// Here this actual rotation is computed for each point. These can be used to
// complement consecutive sweeps.
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
    double horizontalAngle = atan2(point(1), point(0));
    double verticalAngle   = atan(point(2) / Eigen::Vector2d(point(0), point(1)).norm());
    if(position < -1 && horizontalAngle >  1) horizontalAngle -= 2 * M_PI;
    if(position >  1 && horizontalAngle < -1) horizontalAngle += 2 * M_PI;
    outFile << horizontalAngle << " " << verticalAngle << std::endl;
  }

  return 0;
}

