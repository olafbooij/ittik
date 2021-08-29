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
  auto& prev_pose = poses.front();
  for(auto& pose: poses)
  {
    liespline::plot_se3(pose, file, .05);
    std::cout << liespline::logse3(prev_pose.inverse() * pose).transpose() / (2 * M_PI) << std::endl;
    prev_pose = pose;
  }
  return 0;
}

