#include<iostream>
#include<fstream>
#include<string>
#include<boost/format.hpp>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include"util.hpp"

//#include<unapply_calibration_sweep.hpp>

auto read_times(const std::string filename)
{
  std::vector<double> times;
  std::string date, time;
  std::ifstream file(filename);
  while(file >> date >> time)
  {
    int h, m;
    double s;
    sscanf(time.c_str(), "%d:%d:%lf", &h, &m, &s);
    times.emplace_back(((h * 60) + m) * 60 + s);
  }
  return times;
}
struct velo_time{double start; double end;};
auto read_velo_times(const std::string base_path)
{
  std::vector<velo_time> times;
  auto start_times = read_times(base_path + "velodyne_points/timestamps_start.txt");
  auto end_times   = read_times(base_path + "velodyne_points/timestamps_end.txt");
  if(start_times.size() != end_times.size())
    std::cout << "start_times.size() != end_times.size()" << std::endl;
  for(int i = 0; i < start_times.size(); ++i)
    times.emplace_back(velo_time{start_times.at(i), end_times.at(i)});
  return times;
}
auto read_pose_times(const std::string base_path)
{
  return read_times(base_path + "oxts/timestamps.txt");
}

using Pose = Eigen::Isometry3d;
auto read_imu_velo_calib(const std::string base_path)
{
  Pose cal;
  std::ifstream file(base_path + "calib_imu_to_velo.txt");
  std::string ignore;
  file >> ignore >> ignore >> ignore; // header with calib_time

  auto& m = cal.matrix();
  m.row(3).setZero();
  m(15) = 1;

  file >> ignore; // R:
  file >> m(0) >> m(4) >> m(8)
       >> m(1) >> m(5) >> m(9)
       >> m(2) >> m(6) >> m(10);

  file >> ignore; // T:
  file >> m(12) >> m(13) >> m(14);

  return cal;
}

auto mercator(double lat, double lon)
{
  auto latrad = lat * M_PI / 180.;
  auto lonrad = lon * M_PI / 180.;
  static auto scale = cos(latrad); // hacky !
  auto er = 6378137.;  // earth radius (approx.) in meters
  auto tx = scale * er * lonrad;
  auto ty = scale * er * log(tan(M_PI / 4. + latrad / 2));
  return std::pair(tx, ty);
}

struct timed_pose{double time; Pose pose;};
auto read_poses(const std::string base_path, const auto& pose_time)
{
  std::vector<timed_pose> poses;
  for(auto [pose_i, time]: enumerate(pose_time))
  {
    auto filename = boost::format("%010d.txt") % pose_i;
    std::ifstream file(base_path + "oxts/data/" + filename.str());
    double lat, lon, alt, rol, pit, yaw;
    file >> lat >> lon >> alt >> rol >> pit >> yaw;
    // from pykitti/utils.py :
    Eigen::AngleAxisd rolR(rol, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitR(pit, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawR(yaw, Eigen::Vector3d::UnitZ());
    auto rotation = yawR * pitR * rolR;
    auto [tx, ty] = mercator(lat, lon);
    Eigen::Translation3d translation(tx, ty, alt);
    poses.emplace_back(timed_pose{time, Pose(translation * rotation)});
  }
  return poses;
}


int main(int argc, char* argv[])
{
  using namespace std;
  //ifstream sweepFile(argv[1]);
  //ofstream outFile(argv[4]);

  // let's do it hacky:
  // read pose and timing data (oxts + lidar time)
  // approximate time based on heading of point (which has quite a big error of course)
  // (first just assumed 2pi, later determine the heading-change using pose-date)
  // linearly interpolate pose and unapply

  std::string day_path("../kitti/2011_09_30/");
  std::string data_path(day_path + "2011_09_30_drive_0016_extract/");
  auto imu_p_lidar = read_imu_velo_calib(day_path);
  auto velo_time = read_velo_times(data_path);
  auto pose_time = read_pose_times(data_path);
  auto poses = read_poses(data_path, pose_time);
  std::vector<Pose> rel_poses;
  auto zero_pose = poses.front().pose;
  for(auto [time, pose]: poses)
    rel_poses.emplace_back(zero_pose.inverse() * pose);

  ifstream sweepFile("00_odo.txt");
  int sweep_id = 3; // = 0 in odmetry
  auto sweep_time = velo_time.at(sweep_id);

  auto delta = .75;
  auto sweep_ref_time = (1 - delta) * sweep_time.start + delta * sweep_time.end;
  // ... I should interpolate
  auto world_p_imu = std::lower_bound(poses.begin(), poses.end(), time, [](auto pose, auto time){return pose.time < time;})->pose;

  Eigen::Vector3d point;
  double refl;
  while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
  {
    // correct point ... get lidar position
    auto hori_angle = atan2(velo.p(1), velo.p(0));
    auto delta = hori_angle / (2 * M_PI);
    auto time = (1 - delta) * sweep_time.start + delta * sweep_time.end;
    auto world_p_imu = std::lower_bound(poses.begin(), poses.end(), time, [](auto pose, auto time){return pose.time < time;})->pose;

    auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
    outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << std::endl;
  }

  return 0;
}

