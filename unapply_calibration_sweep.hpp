#include<fstream>
#include<iostream>
#include<Eigen/Core>

#include<calibration_kitti.hpp>
#include<unapply_calibration.hpp>

struct Probe
{
  double distance;
  double intensity;
};

// get the probe order based vertical angle
auto determine_probe_order(const Calibration calibration)
{
  std::array<std::size_t, calibration.size()> perm;
  for(size_t i = 0; i < calibration.size(); ++i) perm.at(i) = i;
  std::sort(perm.begin(), perm.end(), [&calibration](auto a, auto b){return calibration.at(a).vertCorrection < calibration.at(b).vertCorrection;});
  return perm;
}

int main(int argc, char* argv[])
{
  auto probe_order = determine_probe_order(kitti_probe_calibration());
  std::cout << probe_order.size() << std::endl;
  for(auto probe_id: probe_order)
    std::cout << probe_id << std::endl;
}

