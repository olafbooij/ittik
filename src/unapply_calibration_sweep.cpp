#include<fstream>
#include<Eigen/Core>

#include<unapply_calibration_sweep.hpp>

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


// Read in a sweep file from the KITTI dataset and outputs raw measurement data (ignoring reflectivity values).
// example usage:
// ./unapply_calibration_sweep $KITTIDIR/2011_09_26/2011_09_26_drive_0002_extract/velodyne_points/data/0000000071.txt ittiked.sweep
int main(int argc, char* argv[])
{
  SweepUncalibrator sweepUncalibrator;

  std::ifstream sweepFile(argv[1]);
  std::ofstream outFile(argv[2]);
  auto calibration = kitti_probe_calibration();

  double onestep = 2 * M_PI / 4000;

  //Eigen::MatrixXi image(64, 4000 + maxpixel - minpixel);
  Eigen::MatrixXi image(64, 4000);
  int pointId = 0;
  Eigen::Vector3d point;
  double refl;
  while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
    long pix = std::lround(position / onestep) + 1999;
    if(pix == -1) pix = 3999;
    //pix += std::lround(calibration.at(probeId).rotCorrection / onestep) - minpixel;
    image(vertId_, pix) = pointId;
    outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << " " << pix << std::endl;
    ++pointId;
  }
  int targetI = 0;
  for(int colI = 0; colI < image.cols(); ++colI)
    if(image.col(colI).sum() > 0)
      image.col(targetI++) = image.col(colI);
  write_pgm(image, std::ofstream("_unap.pgm"));

  std::vector<int> shifts;
  for(auto laser_calibration: calibration)
    shifts.emplace_back(std::lround(laser_calibration.rotCorrection / onestep * targetI / image.cols()));
  auto laserspread = std::minmax_element(shifts.begin(), shifts.end());
  auto minpixel = *(laserspread.first);
  auto maxpixel = *(laserspread.second);
  std::cout << minpixel << " " << maxpixel << std::endl;

  Eigen::MatrixXi image_shifted(64, targetI + (maxpixel - minpixel));
  for(int rowI = 0; rowI < image.rows(); ++rowI)
  {
    auto probeId = determine_probe_order(kitti_probe_calibration()).at(rowI);
    //image.block(rowI, 0, 1, std::lround(calibration.at(probeId).rotCorrection / onestep)).setZero();
    image_shifted.block(rowI, - minpixel + shifts.at(probeId), 1, targetI) = image.block(rowI, 0, 1, targetI);
    //image.block(rowI, , 1, ).setZero();
  }
  write_pgm(image_shifted, std::ofstream("_unap_shifted.pgm"));
  return 0;
}

