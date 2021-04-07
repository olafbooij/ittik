#include"io.hpp"
#include"liespline/se3_plot.hpp"

int main(int argc, char* argv[])
{
  using namespace ittik;
  std::string raw_data_path(argv[1]);
  auto imu_p_lidar = read_imu_velo_calib(raw_data_path + "../");
  auto pose_time = read_pose_times(raw_data_path);
  auto poses = read_poses(raw_data_path, pose_time);
  {
    Pose origin = poses.front().pose.inverse();
    for(auto& [time, pose]: poses)
      // HACKING
      pose = imu_p_lidar.inverse() * origin * pose * imu_p_lidar;
  }
  std::ofstream file(argv[2]);
  for(auto& [time, pose]: poses)
    liespline::plot_se3(pose, file, .05);

  return 0;
}

