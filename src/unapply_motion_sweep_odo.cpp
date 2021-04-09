#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>
#include<boost/format.hpp>

#include"io.hpp"
#include"se3spline.hpp"
#include<unapply_calibration_sweep.hpp>

#include"liespline/se3_plot.hpp"

int main(int argc, char* argv[])
{
  using namespace std;
  using namespace ittik;
  std::string odometryPath(argv[1]);
  int datasetId(atoi(argv[2]));
  int sweepId(atoi(argv[3]));
  std::ofstream outFile(argv[4]);

  auto datasetPath = odometryPath + "/sequences/" + (boost::format("%02d") % datasetId).str()
  auto sweepFileName = datasetPath + "/velodyne/" + (boost::format("%06d.bin") % sweepId).str();

  // let's do it hacky:
  // read poses from odometry dataset
  // approximate time based on heading of point (which has quite a big error of course)
  // (first just assumed 2pi, later determine the heading-change using pose-date)
  // linearly interpolate pose and unapply

  auto cam_p_lidar = read_odometry_calib(datasetPath + "calib.txt");
  auto allPoses = read_odometry_poses(odometryPath + (boost::format("/%02d.txt") % datasetId).str());
  decltype(allPoses) poses(allPoses.at(sweepId-2), allPoses.at(sweepId+3));

  auto interpolate_pose = [](auto poses, auto sweep_time, auto delta)
  {
    // TODO
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
    //liespline::plot_se3(interpolatedPose, file, .05);
    //filep << interpolatedPose.translation().transpose() << " " << scan_time - poses.front().time << std::endl;
    return interpolatedPose;
  };
  auto world_p_imu_ref = interpolate_pose(poses, sweep_time, .5);
  auto world_p_lidar_ref = world_p_imu_ref * imu_p_lidar;

  ifstream sweepFile(sweepFileName);
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

