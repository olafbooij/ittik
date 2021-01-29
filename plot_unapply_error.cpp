#include<fstream>
#include<Eigen/Core>

#include<unapply_calibration_sweep.hpp>
#include<apply_calibration.hpp>

int main(int argc, char* argv[])
{
  SweepUncalibrator sweepUncalibrator;

  std::ifstream sweepFile(argv[1]);
  std::ofstream outFile(argv[2]);
  Eigen::Vector3d point;
  double refl;

  double distance_error = 0;
  double angle_error = 0;
  double norm = 0;
  std::array<double, 64> norms; norms.fill(0.);
  std::array<std::size_t, 64> counts; counts.fill(0);
  std::size_t n = 0;
  while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);

    auto pointAgain = apply_calibration(position, distanceUncor, kitti_probe_calibration().at(probeId));
    std::cout.precision(7);
    auto normrad = [](auto v){return v<M_PI?v:2*M_PI-v;};
    //outFile << probeId << " " << point.transpose() << " " << position << " " << distanceUncor << " " << (point - pointAgain).norm()
    //        << " " << fabs(point.norm() - pointAgain.norm())
    //        << " " << normrad(fabs(atan2(point(0), point(1)) - atan2(pointAgain(0), pointAgain(1))))
    //        << std::endl;
    auto runmean = [&n](auto& m, auto v){m = (n * m + v) / (n + 1);};
    auto runmeanc = [](auto& m, auto v, auto n){m = (n * m + v) / (n + 1);};
    ++n;
    runmean(distance_error, fabs(point.norm() - pointAgain.norm()));
    runmean(norm, (point - pointAgain).norm());
    runmean(angle_error, normrad(fabs(atan2(point(0), point(1)) - atan2(pointAgain(0), pointAgain(1)))));
    ++counts[vertId_];
    runmeanc(norms[vertId_], (point - pointAgain).norm(), counts[vertId_]);
  }
  std::cout << norm << std::endl;
  std::cout << distance_error << std::endl;
  std::cout << angle_error << std::endl;

  for(auto norm: norms) outFile << norm << std::endl;

  return 0;
}


