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
  //  std::cout << probeId << " " << lasers.size() << std::endl << std::endl << std::endl;
  //  for(auto point: lasers)
  //    std::cout << unapply_calibration(point, kitti_probe_calibration().at(probeId)).first << std::endl;
  //}

  std::array<decltype(sweep.front().begin()), 64> lasersAtPose;
  for(auto [probeId, lasers]: enumerate(sweep))
    lasersAtPose.at(probeId) = lasers.begin();

  // take one rotation step, check which next measurements fit.
  // do this for say 100 steps, or until there's 2 or 3 times none fitting...
  // fit a pose using pnp on all 100 steps
  // next 100 steps (or sliding window).



  // step to future and past as follows
  // predict heading of measurement
  // match actual measurement
  // estimate adjustment of pose given matched measurements
  // perhaps not use oxts data, only to compare pose found 

  // how to estimate adjustment...
  // it is perspective-n-point: N 3d points and I know where they should be measured 
  // one linear step (in some space) would suffice...
  // check on raw data, there sensor should be standing still.





  return 0;
}

