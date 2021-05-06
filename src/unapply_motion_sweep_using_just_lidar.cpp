#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>

#include"io.hpp"
#include"util.hpp"
#include<unapply_calibration_sweep.hpp>
#include<unapply_calibration_sweep_Triess.hpp>

#include"liespline/se3_plot.hpp"

auto readSweep(auto&& file)
{
  SweepUncalibratorTriess sweepUncalibrator; // only using the reading part
  std::array<std::vector<Eigen::Vector3d>, 64> sweep;
  Eigen::Vector3d point;
  double refl;
  while(file >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto probeId = sweepUncalibrator.get_probeId(point);
    assert(probeId < 65);
    sweep.at(probeId).emplace_back(point);
  }
  for(const auto& lasers: sweep) assert(! lasers.empty());
  return sweep;
}


int main(int argc, char* argv[])
{
  using namespace ittik;
  //ifstream sweepFile(argv[1]);
  //ofstream outFile(argv[4]);

  // take two
  // read all lidar data
  const auto sweep = readSweep(std::ifstream("00_odo.txt"));

  //for(auto [probeId, lasers]: enumerate(sweep))
  //{
  //  //std::cout << probeId << " " << lasers.size() << std::endl << std::endl << std::endl;
  //  //for(auto point: lasers)
  //  for(auto pointI = 0; pointI < 100; ++pointI)
  //  {
  //    auto [position, distanceUncor] = unapply_calibration(lasers.at(pointI), kitti_probe_calibration().at(probeId));
  //    std::cout << probeId << " " << position << " " << distanceUncor << std::endl;
  //  }
  //}

  std::array<decltype(sweep.front().begin()), 64> lasersAtPoseStart;
  std::array<decltype(sweep.front().begin()), 64> lasersAtPoseEnd;
  for(auto [probeId, lasers]: enumerate(sweep))
    lasersAtPoseEnd.at(probeId) = lasersAtPoseStart.at(probeId) = lasers.begin();

  // take next points up to .05 rad away from current angle per laser
  // perhaps should iterate until error gets to large
  for(auto [probeId, lasers]: enumerate(sweep))
  {
    auto& laserEnd = lasersAtPoseEnd.at(probeId);
    while(atan2((*laserEnd)(1), (*laserEnd)(0)) < .1)
      ++laserEnd;
    //std::cout << laserEnd - lasersAtPoseStart.at(probeId) << std::endl;
  }

  // unapply and check if they match with a ground truth measurement
  double stepangle = 2 * M_PI / 4000;
  for(auto [probeId, lasers]: enumerate(sweep))
  {
    for(auto laserIt = lasersAtPoseStart.at(probeId); laserIt != lasersAtPoseEnd.at(probeId); ++laserIt)
    {
      auto [position, distanceUncor] = unapply_calibration(*laserIt, kitti_probe_calibration().at(probeId));
      auto error = position - std::lround(position / stepangle) * stepangle;
      std::cout << probeId << " " << error << std::endl;

    }
  }

  // estimate pose (pnp) using matches
  //   it is perspective-n-point: N 3d points and I know where they should be measured 
  //   one linear step (in some space) would suffice...

  return 0;
}

