#include<fstream>
#include<Eigen/Core>

#include"unapply_calibration_sweep.hpp"

// Read in a sweep file from the KITTI dataset and outputs raw measurement data (ignoring reflectivity values).
// example usage:
// ./unapply_calibration_sweep $KITTIDIR/2011_09_26/2011_09_26_drive_0002_extract/velodyne_points/data/0000000071.txt ittiked.sweep
int main(int argc, char* argv[])
{
  auto calibration = kitti_probe_calibration();

  double stepangle = 2 * M_PI / 4000;

  // find non-empty columns
  std::array<int, 4000> points_per_column{};
  {
    SweepUncalibrator sweepUncalibrator;
    std::ifstream sweepFile(argv[1]);
    Eigen::Vector3d point;
    double refl;
    while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
    {
      auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
      long pix = std::lround(position / stepangle) + 1999;
      if(pix == -1) pix = 3999;
      ++points_per_column.at(pix);
    }
  }

  // get a mapping from column to x-coordinate
  std::array<int, 4000> column_x{};
  {
    int x = 0;
    for(int colI = 0; colI < points_per_column.size(); ++colI)
      if(points_per_column.at(colI) > 0)
        column_x.at(colI) = x++;
  }

  // get discrete shift per laser
  std::vector<int> shifts;
  for(auto laser_calibration: calibration)
    shifts.emplace_back(-std::lround(
    unapply_calibration(Eigen::Vector3d(20.,0.,0.), laser_calibration).first // assumption that objects are 20 meters away
    / stepangle * (column_x.back() + 1) / column_x.size()));
  auto laserspread = std::minmax_element(shifts.begin(), shifts.end());
  auto minpixel = *(laserspread.first);
  auto maxpixel = *(laserspread.second);
  std::cout << minpixel << " " << maxpixel << std::endl;

  // determine for each point its pixel coordinates
  {
    std::ofstream outFile(argv[2]);
    // repetez maintenant
    SweepUncalibrator sweepUncalibrator;
    std::ifstream sweepFile(argv[1]);
    Eigen::Vector3d point;
    double refl;
    while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
    {
      auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
      long pix = std::lround(position / stepangle) + 1999;
      if(pix == -1) pix = 3999;
      outFile << column_x.at(pix) - minpixel + shifts.at(probeId) << " " << vertId_ << std::endl; // could also do the column changes in column_x
    }
  }

  return 0;
}

