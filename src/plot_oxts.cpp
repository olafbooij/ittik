#include"io.hpp"
#include"liespline/se3_plot.hpp"

int main(int argc, char* argv[])
{
  using namespace ittik;
  auto pose_time = read_pose_times(argv[1]);
  auto poses = read_poses(argv[1], pose_time);
  {
    Pose origin = poses.front().pose.inverse();
    for(auto& [time, pose]: poses)
      pose = origin * pose;
  }
  std::ofstream file(argv[2]);
  for(auto& [time, pose]: poses)
    liespline::plot_se3(pose, file, .05);

  return 0;
}

