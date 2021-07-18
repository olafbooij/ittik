#include<fstream>
#include<Eigen/Core>

#include"image_writing.hpp"
#include"unapply_calibration_sweep.hpp"

int main(int argc, char* argv[])
{
  using namespace ittik;

  std::ifstream coordsFile(argv[1], std::ios::in | std::ios::binary);
  std::ifstream sweepFile(argv[2]);
  std::ofstream ppmFile(argv[3]);

  Eigen::MatrixXi imageR(64, 2282); imageR.setZero();
  Eigen::MatrixXi imageG(64, 2282); imageG.setZero();
  Eigen::MatrixXi imageB(64, 2282); imageB.setZero();
  while(coordsFile.good() && !coordsFile.eof())
  {
    auto readunint16 = [](auto& file){
      uint16_t var;
      file.read(reinterpret_cast<char*>(&var), sizeof(var));
      return var;
    };
    auto x = 2281 - readunint16(coordsFile);
    auto y = readunint16(coordsFile);
    assert(x < 2282);
    assert(sweepFile.good());
    Eigen::Vector3d point;
    double refl;
    sweepFile >> point(0) >> point(1) >> point(2) >> refl;

    double range = 1 - 3 / point.norm();
    imageR(y, x) = static_cast<int>(floor(255 * sqrt(range)));
    imageG(y, x) = static_cast<int>(floor(255 * pow(range, 3)));
    imageB(y, x) = static_cast<int>(floor(255 * sin(2*M_PI * range)));
  }
  write_ppm(imageR,
            imageG,
            imageB, ppmFile);
  return 0;
}

