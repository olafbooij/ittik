#include"io.hpp"
#include"liespline/se3_plot.hpp"

int main(/*int argc, char* argv[]*/)
{
  using namespace ittik;
  std::string data_path("kitti/2011_09_302011_09_30_drive_0016_extract/");
  auto pose_time = read_pose_times(data_path);
  auto poses = read_poses(data_path, pose_time);
  std::ofstream file("oxts.poses");
  for(auto& [time, pose]: poses)
    liespline::plot_se3(pose, file, .05);

  return 0;
}

