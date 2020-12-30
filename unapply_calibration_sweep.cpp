#include<fstream>
#include<Eigen/Core>

#include<unapply_calibration_sweep.hpp>


int main(int argc, char* argv[])
{
  Sweep_uncalibrator sweep_uncalibrator;

  std::ifstream sweep_file(argv[1]);
  std::ofstream out_file(argv[2]);
  Eigen::Vector3d point;
  double refl;

  while(sweep_file >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto probe_data = sweep_uncalibrator(point);
    out_file << std::get<0>(probe_data) << " " << std::get<1>(probe_data) << " " << std::get<2>(probe_data) << std::endl;
  }

  return 0;
}

