#include<fstream>
#include<tuple>
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

struct Sweep_uncalibrator
{
  int vert_id_;
  double horizontal_angle_prev_;
  std::array<std::size_t, 64> probe_order_;
  Sweep_uncalibrator() :
    vert_id_(0),
    horizontal_angle_prev_(0),
    probe_order_(determine_probe_order(kitti_probe_calibration()))
  {}

  auto operator()(const Eigen::Vector3d& point)
  {
    {
      double horizontal_angle = atan2(point(1), point(0));
      if(horizontal_angle < 0)
        horizontal_angle += 2 * M_PI;
      if(horizontal_angle < horizontal_angle_prev_)
        ++vert_id_;
      horizontal_angle_prev_ = horizontal_angle;
    }
    auto probe_id = probe_order_.at(vert_id_);
    auto probe_data = pointToMeasurement(point, kitti_probe_calibration().at(probe_id));

    return std::make_tuple(probe_id, probe_data.first, probe_data.second);
  }

};

int main(int argc, char* argv[])
{
  Sweep_uncalibrator sweep_uncalibrator;

  std::ifstream sweep_file(argv[1]);
  std::ofstream out_file(argv[2]);
  Eigen::Vector3d point;
  double refl;

  while(sweep_file >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto probe_data = sweep_uncalibrator(point);
    out_file << std::get<0>(probe_data) << " " << std::get<1>(probe_data) << " " << std::get<2>(probe_data) << std::endl;
  }

  return 0;
}

