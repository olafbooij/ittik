#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>

#include"io.hpp"
#include"util.hpp"
#include"unapply_calibration_sweep.hpp"
#include"unapply_calibration_sweep_Triess.hpp"

#include"liespline/se3_plot.hpp"

#include"optimize_lidar_pose_euclid.hpp"


struct Point
{
  Eigen::Vector3d point;
  std::size_t     probeId;
  double          hori_angle;
  double          position;
  double          distanceUncor;
};

auto readSweep(auto&& file)
{
  SweepUncalibratorTriess sweepUncalibrator; // only using the reading part
  std::vector<Point> points;
  Eigen::Vector3d point;
  double refl;
  while(file >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
    assert(probeId < 65);
    points.emplace_back(Point{point, probeId, atan2(point(1), point(0)), position, distanceUncor});
  }
  return points;
}


int main(int argc, char* argv[])
{
  using namespace ittik;
  std::ifstream raw_sweep_file(argv[1]);
  std::ifstream corrected_sweep_file(argv[2]);
  //std::ofstream outFile(argv[3]);

  const auto raw_sweep = readSweep(raw_sweep_file);
  assert(! raw_sweep.empty());
  const auto corrected_sweep = readSweep(corrected_sweep_file);
  assert(corrected_sweep.size() == raw_sweep.size());

  // start from position 0.
  // get for each probe the 1 - 2 - nan (fingerprint)
  // somehow match them...
  // get 5 - 6 - 4 - nan (snipsel)
  // match unique combi (perhaps 7 snipsels (?))

  // or
  // fill columns of non-fires
  // per piont angle of up to 10 degrees or so, either localize, by matching,
  // or fill if matched

  std::array<std::vector<int>, 64> fingerprints;
  for(int pointI = 1; pointI < corrected_sweep.size(); ++pointI)
  {
    auto& point = corrected_sweep.at(pointI);
    auto& prev_= corrected_sweep.at(pointI - 1);
    // determine correspondences based on original cloud
    auto [position, distanceUncor] = unapply_calibration(point.point, kitti_probe_calibration().at(point.probeId));
    auto [prev_pos, prev_distance] = unapply_calibration(prev_.point, kitti_probe_calibration().at(point.probeId));
    auto delta = position - prev_pos;
    int fp = 0;
    if(fabs(distanceUncor - prev_distance) < .5)
    {
      if(fabs(delta - M_PI / 2000) < .001)
        fp = 1;
      if(fabs(delta - M_PI / 1000) < .001)
        fp = 2;
    }
    fingerprints.at(raw_sweep.at(pointI).probeId).emplace_back(fp);
    std::cout << fp << " " << raw_sweep.at(pointI).probeId << " " << raw_sweep.at(pointI).position << std::endl;
  }

  return 0;
}

