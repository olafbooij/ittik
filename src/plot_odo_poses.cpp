#include"io.hpp"
#include"liespline/se3_plot.hpp"

int main(int argc, char* argv[])
{
  using namespace ittik;
  auto poses = read_odometry_poses(argv[1]);
  auto cam_p_lidar = read_odometry_calib(argv[2]);

  for(auto& pose: poses)
    pose = cam_p_lidar.inverse() * pose * cam_p_lidar;

  std::ofstream file(argv[3]);
  for(auto& pose: poses)
    liespline::plot_se3(pose, file, .05);

  return 0;
}

