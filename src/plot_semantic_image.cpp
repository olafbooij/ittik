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

  std::ifstream coordsFile(argv[1]);
  std::ifstream semanticsFile(argv[2], std::ios::in | std::ios::binary);
  std::ofstream ppmFile(argv[3]);

  Eigen::MatrixXi imageR(64, 2280); imageR.setZero();
  Eigen::MatrixXi imageG(64, 2280); imageG.setZero();
  Eigen::MatrixXi imageB(64, 2280); imageB.setZero();
  int x, y;
  while(coordsFile >> x >> y)
  {
    assert(y < 2280);
    uint16_t semantic;
    uint16_t instance;
    semanticsFile.read(reinterpret_cast<char*>(&semantic), sizeof(decltype(semantic)));
    semanticsFile.read(reinterpret_cast<char*>(&instance), sizeof(decltype(instance)));
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
