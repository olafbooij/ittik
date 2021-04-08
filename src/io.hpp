#pragma once
#include<fstream>
#include<string>
#include<optional>
#include<boost/format.hpp>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include"util.hpp"

namespace ittik
{

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
    assert(start_times.size() == end_times.size());
    for(int i = 0; i < start_times.size(); ++i)
      times.emplace_back(velo_time{start_times.at(i), end_times.at(i)});
    return times;
  }
  auto read_pose_times(const std::string base_path)
  {
    return read_times(base_path + "oxts/timestamps.txt");
  }

  using Pose = Eigen::Isometry3d;
  auto read_calib(const std::string filename)
  {
    Pose cal;
    std::ifstream file(filename);
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
    assert(file.good());

    return cal;
  }
  auto read_imu_velo_calib(const std::string base_path)
  {
    return read_calib(base_path + "calib_imu_to_velo.txt");
  }

  template<typename File>
  std::optional<Pose> read_odometry_pose(File&& file)
  {
    Pose pose;
    auto& m = pose.matrix();
    m.row(3).setZero();
    m(15) = 1;
    if(file >> m(0) >> m(4) >> m(8)  >> m(12)
            >> m(1) >> m(5) >> m(9)  >> m(13)
            >> m(2) >> m(6) >> m(10) >> m(14))
      return pose;
    else
      return std::nullopt;
  }

  auto read_odometry_poses(const std::string filename)
  {
    std::vector<Pose> poses;
    std::ifstream file(filename);
    while(auto pose = read_odometry_pose(file))
      poses.push_back(*pose);
    return poses;
  }

  auto read_odometry_calib(const std::string filename)
  {
    std::ifstream file(filename);
    std::string ignore;
    for(int i = 13 * 4 + 1; i--;) // 4 projection matrices and Tr:
      file >> ignore;
    auto pose = read_odometry_pose(file);
    return *pose;
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

}

