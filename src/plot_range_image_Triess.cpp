#include<fstream>
#include<Eigen/Core>

#include"image_writing.hpp"
#include"unapply_calibration_sweep.hpp"

int main(int argc, char* argv[])
{
  using namespace ittik;

  std::ifstream unapFile(argv[1]);
  std::ifstream sweepFile(argv[2]);
  std::ofstream ppmFile(argv[3]);

  Eigen::MatrixXi imageR(64, 2282); imageR.setZero();
  Eigen::MatrixXi imageG(64, 2282); imageG.setZero();
  Eigen::MatrixXi imageB(64, 2282); imageB.setZero();
  int probeId, vertId;
  double position, dist;
  std::size_t overlap = 0;
  while(unapFile >> probeId >> position >> dist >> vertId)
  {
    assert(sweepFile.good());
    Eigen::Vector3d point;
    double refl;
    sweepFile >> point(0) >> point(1) >> point(2) >> refl;

    int y = vertId;
    int x = 2099 - static_cast<int>(floor((atan2(point(1), point(0)) / (2 * M_PI) + .5) * 2100));
    assert(x < 2282);

    double range = 1 - 3 / point.norm();
    imageR(y, x) = static_cast<int>(floor(255 * sqrt(range)));
    imageG(y, x) = static_cast<int>(floor(255 * pow(range, 3)));
    imageB(y, x) = static_cast<int>(floor(255 * sin(2*M_PI * range)));

    if(imageR(y, x)) ++overlap;
  }
  std::cout << overlap << std::endl;
  write_ppm(imageR,
            imageG,
            imageB, ppmFile);
  return 0;
}

