#include<fstream>
#include<Eigen/Core>

#include"image_writing.hpp"
#include"unapply_calibration_sweep.hpp"

// Create image (ppm) given a pixel coordinate file (made using discretize_sweep) and a semantics label file.
// example usage:
// ./unapply_calibration_sweep coords_file $SEMANTICKITTIDIR/sequences/04/labels/000000.label image.ppm
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
    auto x = readunint16(coordsFile);
    auto y = readunint16(coordsFile);
    assert(x < 2282);
    assert(sweepFile.good());
    Eigen::Vector3d point;
    double refl;
    sweepFile >> point(0) >> point(1) >> point(2) >> refl;
    //double range = 1 - point.norm() / 50;
    double range = 1 - 3 / point.norm();
//      0: 0               1: 0.5             2: 1
//             3: x               4: x^2             5: x^3
//             6: x^4             7: sqrt(x)         8: sqrt(sqrt(x))
//             9: sin(90x)       10: cos(90x)       11: |x-0.5|
//            12: (2x-1)^2       13: sin(180x)      14: |cos(180x)|
//            15: sin(360x)      16: cos(360x)      17: |sin(360x)|
//            18: |cos(360x)|    19: |sin(720x)|    20: |cos(720x)|
//            21: 3x             22: 3x-1           23: 3x-2
//            24: |3x-1|         25: |3x-2|         26: (3x-1)/2
//            27: (3x-2)/2       28: |(3x-1)/2|     29: |(3x-2)/2|
//            30: x/0.32-0.78125 31: 2*x-0.84       32: 4x;1;-2x+1.84;x/0.08-11.5
//            33: |2*x - 0.5|    34: 2*x            35: 2*x - 0.5

// 7,5,15
//sqrt(x), x^3, sin(360x)
   
    imageR(y, x) = static_cast<int>(floor(255 * sqrt(range)));
    imageG(y, x) = static_cast<int>(floor(255 * pow(range, 3)));
    imageB(y, x) = static_cast<int>(floor(255 * sin(2*M_PI * range)));
  }
  write_ppm(imageR,
            imageG,
            imageB, ppmFile);
  return 0;
}

