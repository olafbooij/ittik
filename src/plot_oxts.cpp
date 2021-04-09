#include"io.hpp"
#include"liespline/se3_plot.hpp"

int main(int argc, char* argv[])
{
  using namespace ittik;
  std::string raw_data_path(argv[1]);
  auto imu_p_lidar = read_imu_velo_calib(raw_data_path + "../");
  auto pose_time = read_pose_times(raw_data_path);
  auto poses = read_poses(raw_data_path, pose_time);
  //decltype(poses) poses_(poses.begin() + 45, poses.end());
  //poses = poses_;
  {
    for(auto& [time, pose]: poses)
      pose =  pose * imu_p_lidar;
    Pose origin = poses.front().pose.inverse();
    for(auto& [time, pose]: poses)
      pose = origin * pose;
  }
  std::ofstream file(argv[2]);
  for(auto& [time, pose]: poses)
    liespline::plot_se3(pose, file, .05);

  return 0;
}

