#include<iostream>

#include<unapply_calibration_sweep.hpp>

int main(/*int argc, char* argv[]*/)
{
  for(auto probe_id: determine_probe_order(kitti_probe_calibration()))
    std::cout << probe_id << std::endl;

  return 0;
}

