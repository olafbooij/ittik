#include<fstream>
#include<Eigen/Core>

#include"semantics.hpp"
#include"image_writing.hpp"
#include"unapply_calibration_sweep.hpp"

// Create image (ppm) given a pixel coordinate file (made using discretize_sweep) and a semantics label file.
// example usage:
// ./unapply_calibration_sweep coords_file $SEMANTICKITTIDIR/sequences/04/labels/000000.label image.ppm
int main(int argc, char* argv[])
{
  using namespace ittik;

  std::ifstream coordsFile(argv[1], std::ios::in | std::ios::binary);
  std::ifstream semanticsFile(argv[2], std::ios::in | std::ios::binary);
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
    auto x = readunint16(coordsFile);
    auto y = readunint16(coordsFile);
    assert(x < 2282);
    assert(semanticsFile.good());
    auto semantic = readunint16(semanticsFile);
    auto instance = readunint16(semanticsFile);
    auto [r, g, b] = color_map[semantic];
    imageR(y, x) = r;
    imageG(y, x) = g;
    imageB(y, x) = b;
  }
  write_ppm(imageR,
            imageG,
            imageB, ppmFile);
  return 0;
}

