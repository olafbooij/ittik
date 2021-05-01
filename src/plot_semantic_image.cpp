#include<fstream>
#include<Eigen/Core>

#include"semantics.hpp"
#include"unapply_calibration_sweep.hpp"

template<typename ImageT, typename File>  // e.g. Eigen::MatrixXd, std::cout
void write_pgm(const ImageT& image, File&& file)
{
  // header
  file << "P2\n" << image.cols() << " " << image.rows() << "\n255\n";
  for(int y = 0; y < image.rows(); ++y)
  {
    for(int x = 0; x < image.cols(); ++x)
      file << std::clamp(static_cast<int>(image(y, x)), 0, 255) << " ";
    file << "\n";
  }
}

template<typename ImageT, typename File>  // e.g. Eigen::MatrixXd, std::cout
void write_ppm(const ImageT& imageR,
               const ImageT& imageG,
               const ImageT& imageB, File&& file)
{
  // header
  file << "P3\n" << imageR.cols() << " " << imageR.rows() << "\n255\n";
  for(int y = 0; y < imageR.rows(); ++y)
  {
    for(int x = 0; x < imageR.cols(); ++x)
    {
      file << std::clamp(static_cast<int>(imageR(y, x)), 0, 255) << " ";
      file << std::clamp(static_cast<int>(imageG(y, x)), 0, 255) << " ";
      file << std::clamp(static_cast<int>(imageB(y, x)), 0, 255) << " ";
    }
    file << "\n";
  }
}


// Create image (ppm) given a pixel coordinate file (made using discretize_sweep) and a semantics label file.
// example usage:
// ./unapply_calibration_sweep coords_file $SEMANTICKITTIDIR/sequences/04/labels/000000.label image.ppm
int main(int argc, char* argv[])
{
  using namespace ittik;

  std::ifstream coordsFile(argv[1], std::ios::in | std::ios::binary);
  std::ifstream semanticsFile(argv[2], std::ios::in | std::ios::binary);
  std::ofstream ppmFile(argv[3]);

  const int image_width = 2300;

  Eigen::MatrixXi imageR(64, image_width); imageR.setZero();
  Eigen::MatrixXi imageG(64, image_width); imageG.setZero();
  Eigen::MatrixXi imageB(64, image_width); imageB.setZero();
  while(coordsFile.good() && !coordsFile.eof())
  {
    auto readunint16 = [](auto& file){
      uint16_t var;
      file.read(reinterpret_cast<char*>(&var), sizeof(var));
      return var;
    };
    auto x = readunint16(coordsFile);
    auto y = readunint16(coordsFile);
    assert(x < image_width);
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

