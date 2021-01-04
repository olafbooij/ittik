#include<iostream>

#include<unapply_calibration_sweep.hpp>

int main(/*int argc, char* argv[]*/)
{
  for(auto probeId: determine_probe_order(kitti_probe_calibration()))
    std::cout << probeId << std::endl;

  return 0;
}
