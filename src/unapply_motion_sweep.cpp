#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>

#include"io.hpp"
#include"se3spline.hpp"
#include<unapply_calibration_sweep.hpp>

#include"liespline/se3_plot.hpp"

int main(int argc, char* argv[])
{
  using namespace std;
  using namespace ittik;
  std::string sweepFileName(argv[1]);
  std::string raw_data_path(argv[2]);
  int sweep_id(atoi(argv[3]));
  ofstream outFile(argv[4]);

  // let's do it hacky:
  // read pose and timing data (oxts + lidar time)
  // approximate time based on heading of point (which has quite a big error of course)
  // (first just assumed 2pi, later determine the heading-change using pose-date)
  // linearly interpolate pose and unapply

  auto imu_p_lidar = read_imu_velo_calib(data_path + "../");
  auto velo_time = read_velo_times(data_path);
  auto pose_time = read_pose_times(data_path);
  auto poses = read_poses(data_path, pose_time);

  {
    Pose origin = poses.front().pose.inverse();
    for(auto& [time, pose]: poses)
      pose = origin * pose;
  }

  {
    std::ofstream file("oxts.positions");
    for(auto& [time, pose]: poses)
      file << pose.translation().transpose() << " " << time - poses.front().time << std::endl;
  }

  // I'm going to assume that oxts timing is constant at 10 ms. Let's check that this is the case
  {
    auto prev = poses.front().time;
    for(auto [time, pose]: std::vector(poses.begin() + 1, poses.end()))
    {
      assert(time - prev - .01  < .001);
      prev = time;
    }
  }

  ifstream sweepFile(sweepFileName);
  auto sweep_time = velo_time.at(sweep_id);

  std::ofstream file("interpolated.poses");
  std::ofstream filep("interpolated.positions");
  auto interpolate_pose = [&file, &filep](auto poses, auto sweep_time, auto delta)
  {
    auto scan_time = (1 - delta) * sweep_time.start + delta * sweep_time.end;
    auto closestPose = std::lower_bound(poses.begin(), poses.end(), scan_time, [](auto pose, auto time){return pose.time < time;}) - 1;
    assert(closestPose - poses.begin() > 0);
    assert(poses.end() -  closestPose > 3);
    // let me split poses for now
    std::vector<Pose> justPoses;
    for(auto [time, pose]: poses)
      justPoses.emplace_back(pose);
    auto closestJustPose = justPoses.begin() + (closestPose - poses.begin());
    auto interpolatedPose = liespline::interpolate<liespline::se3>(closestJustPose - 1, (scan_time - closestPose->time) / ((closestPose + 1)->time - closestPose->time));
    //return closestPose->pose;
    liespline::plot_se3(interpolatedPose, file, .05);
    filep << interpolatedPose.translation().transpose() << " " << scan_time - poses.front().time << std::endl;
    return interpolatedPose;
  };
  auto world_p_imu_ref = interpolate_pose(poses, sweep_time, .5);
  auto world_p_lidar_ref = world_p_imu_ref * imu_p_lidar;

  SweepUncalibrator sweepUncalibrator;
  Eigen::Vector3d point;
  double refl;
  while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
  {
    // correct point ...
    double horizontalAngle = atan2(point(1), point(0));
    auto delta = 1 - (horizontalAngle + M_PI) / (2 * M_PI); // I do not understand this 1 - ...
    auto world_p_imu = interpolate_pose(poses, sweep_time, delta);

    auto lidar_p_lidar_ref = (world_p_imu * imu_p_lidar).inverse() * world_p_lidar_ref;
    Eigen::Vector3d pointCorrected = lidar_p_lidar_ref * point;

    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(pointCorrected);
    outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << std::endl;
  }

  return 0;
}

