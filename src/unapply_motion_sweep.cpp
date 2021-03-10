#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>

#include"io.hpp"
#include"se3spline.hpp"
#include<unapply_calibration_sweep.hpp>

int main(int argc, char* argv[])
{
  using namespace std;
  using namespace ittik;
  //ifstream sweepFile(argv[1]);
  ofstream outFile(argv[4]);

  // let's do it hacky:
  // read pose and timing data (oxts + lidar time)
  // approximate time based on heading of point (which has quite a big error of course)
  // (first just assumed 2pi, later determine the heading-change using pose-date)
  // linearly interpolate pose and unapply

  std::string data_path("../kitti/2011_09_30/2011_09_30_drive_0016_extract/");
  auto imu_p_lidar = read_imu_velo_calib(data_path + "../");
  auto velo_time = read_velo_times(data_path);
  auto pose_time = read_pose_times(data_path);
  auto poses = read_poses(data_path, pose_time);
  for(auto& [time, pose]: poses)
    pose = poses.front().pose.inverse() * pose;

  // I'm going to assume that oxts timing is constant at 10 ms. Let's check that this is the case
  {
    auto prev = poses.front().time;
    for(auto [time, pose]: std::vector(poses.begin() + 1, poses.end()))
    {
      assert(time - prev - .01  < .001);
      prev = time;
    }
  }

  ifstream sweepFile("00_odo.txt");
  int sweep_id = 3; // = 0 in odmetry
  auto sweep_time = velo_time.at(sweep_id);

  auto interpolate_pose = [](auto poses, auto sweep_time, auto delta)
  {
    auto scan_time = (1 - delta) * sweep_time.start + delta * sweep_time.end;
    // taking nearest neighbour... I should interpolate
    //auto closestPose = std::upper_bound(poses.begin(), poses.end(), scan_time, [](auto time, auto pose){return pose.time > time;});
    auto closestPose = std::lower_bound(poses.begin(), poses.end(), scan_time, [](auto pose, auto time){return pose.time < time;}) - 1;
    std::cout << closestPose->time - poses.front().time << " " << scan_time - poses.front().time << std::endl;
    assert(closestPose - poses.begin() > 3);
    assert(poses.end() -  closestPose > 1);
    auto interpolatedPose = liespline::interpolate<liespline::se3>(closestPose - 3, scan_time - closestPose->time);
    //return closestPose->pose;
    return interpolatedPose;
  };
  auto world_p_imu_ref = interpolate_pose(poses, sweep_time, .75);
  auto world_p_lidar_ref = world_p_imu_ref * imu_p_lidar;

  SweepUncalibrator sweepUncalibrator_orig;
  SweepUncalibrator sweepUncalibrator_unmo;
  Eigen::Vector3d point_ref;
  double refl;
  while(sweepFile >> point_ref(0) >> point_ref(1) >> point_ref(2) >> refl)
  {
    // correct point ... get lidar rotational position
    auto [probeId_r, position_r, distanceUncor_r, vertId__r] = sweepUncalibrator_orig(point_ref);
    auto delta = (position_r + M_PI) / (2 * M_PI);
    auto world_p_imu = interpolate_pose(poses, sweep_time, delta);

    //auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
    //outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << std::endl;
    auto lidar_p_lidar_ref = (world_p_imu * imu_p_lidar).inverse() * world_p_lidar_ref;
    Eigen::Vector3d point = lidar_p_lidar_ref * point_ref;
    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator_unmo(point);
    outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << std::endl;
  }

  return 0;
}

