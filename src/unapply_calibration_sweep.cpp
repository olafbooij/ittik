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


// Read in a sweep file from the KITTI dataset and outputs raw measurement data (ignoring reflectivity values).
// example usage:
// ./unapply_calibration_sweep $KITTIDIR/2011_09_26/2011_09_26_drive_0002_extract/velodyne_points/data/0000000071.txt ittiked.sweep
int main(int argc, char* argv[])
{
  using namespace ittik;
  SweepUncalibrator sweepUncalibrator;

  std::ifstream sweepFile(argv[1]);
  std::ifstream semanticsFile(argv[2], std::ios::in | std::ios::binary);
  std::ofstream outFile(argv[3]);
  auto calibration = kitti_probe_calibration();

  double onestep = 2 * M_PI / 4000;

  //Eigen::MatrixXi image(64, 4000 + maxpixel - minpixel);
  Eigen::MatrixXi image(64, 4000); image.setZero();
  Eigen::MatrixXi imageR(64, 4000); imageR.setZero();
  Eigen::MatrixXi imageG(64, 4000); imageG.setZero();
  Eigen::MatrixXi imageB(64, 4000); imageB.setZero();
  Eigen::Vector3d point;
  double refl;
  while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
  {
    uint16_t semantic;
    uint16_t instance;
    semanticsFile.read(reinterpret_cast<char*>(&semantic), sizeof(decltype(semantic)));
    semanticsFile.read(reinterpret_cast<char*>(&instance), sizeof(decltype(instance)));
    auto [r, g, b] = color_map[semantic];
    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
    long pix = std::lround(position / onestep) + 1999;
    if(pix == -1) pix = 3999;
    //pix += std::lround(calibration.at(probeId).rotCorrection / onestep) - minpixel;
    image(vertId_, pix) = 255;
    imageR(vertId_, pix) = static_cast<int>(r);
    imageG(vertId_, pix) = static_cast<int>(g);
    imageB(vertId_, pix) = static_cast<int>(b);
    outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << " " << pix << std::endl;
  }
  int targetI = 0;
  for(int colI = 0; colI < image.cols(); ++colI)
  {
    if(image.col(colI).sum() > 0)
    {
      imageR.col(targetI) = imageR.col(colI);
      imageG.col(targetI) = imageG.col(colI);
      imageB.col(targetI) = imageB.col(colI);
      targetI++;
    }
    }
  write_pgm(imageR, std::ofstream("_unap.pgm"));

  std::vector<int> shifts;
  for(auto laser_calibration: calibration)
    shifts.emplace_back(std::lround(laser_calibration.rotCorrection / onestep * targetI / imageR.cols()));
  auto laserspread = std::minmax_element(shifts.begin(), shifts.end());
  auto minpixel = *(laserspread.first);
  auto maxpixel = *(laserspread.second);

  Eigen::MatrixXi imageR_shifted(64, targetI + (maxpixel - minpixel)); imageR_shifted.setZero();
  Eigen::MatrixXi imageG_shifted(64, targetI + (maxpixel - minpixel)); imageG_shifted.setZero();
  Eigen::MatrixXi imageB_shifted(64, targetI + (maxpixel - minpixel)); imageB_shifted.setZero();
  for(int rowI = 0; rowI < imageR.rows(); ++rowI)
  {
    auto probeId = determine_probe_order(kitti_probe_calibration()).at(rowI);
    //image.block(rowI, 0, 1, std::lround(calibration.at(probeId).rotCorrection / onestep)).setZero();
    imageR_shifted.block(rowI, - minpixel + shifts.at(probeId), 1, targetI) = imageR.block(rowI, 0, 1, targetI);
    imageG_shifted.block(rowI, - minpixel + shifts.at(probeId), 1, targetI) = imageG.block(rowI, 0, 1, targetI);
    imageB_shifted.block(rowI, - minpixel + shifts.at(probeId), 1, targetI) = imageB.block(rowI, 0, 1, targetI);
    //image.block(rowI, , 1, ).setZero();
  }
  write_ppm(imageR_shifted, 
            imageG_shifted,
            imageB_shifted, std::ofstream("_unap_shifted.ppm"));
  return 0;
}

