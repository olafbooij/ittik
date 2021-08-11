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

  // error based on local angle difference
  // get 3 points in a row. if approx the same distance and same
  // position-delta, take difference of position-delta and standard-distance as
  // error. (...could results in outliers...)

  double prev_position = 0;
  double prev_distanceUncor = 0;
  double prev_delta = 0;
  auto prev_point = corrected_sweep.front();
  int okay = 0;
  Eigen::Vector2d error{0., 0.};
  int n = 0;

  for(int pointI = 0; pointI < corrected_sweep.size(); ++pointI)
  {
    auto& point = corrected_sweep.at(pointI);
    // determine correspondences based on original cloud
    auto [position, distanceUncor] = unapply_calibration(point.point, kitti_probe_calibration().at(point.probeId));
    auto delta = position - prev_position;
    if((fabs(distanceUncor - prev_distanceUncor) > .05) ||
       ((fabs(delta - prev_delta) > .001) &&
        (fabs(delta - prev_delta * 2) > .001) &&
        (fabs(delta * 2 - prev_delta) > .001)
       )
      ) // bit of a though one, should maybe make it smaller at the cost of inliers
      okay = 0;
    else
      ++okay;
    prev_position = position;
    prev_distanceUncor = distanceUncor;
    prev_delta = delta;

    if(okay > 1)
    {
      std::cout << raw_sweep.at(pointI).probeId << " " << raw_sweep.at(pointI).position << std::endl;
    }

    prev_point = point;
  }

  return 0;
}

