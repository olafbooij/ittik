#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>

#include"io.hpp"
#include"util.hpp"
#include"unapply_calibration_sweep.hpp"
#include"unapply_calibration_sweep_Triess.hpp"

#include"liespline/se3_plot.hpp"

#include"optimize_lidar_pose_euclid.hpp"

auto readSweep(auto&& file)
{
  SweepUncalibratorTriess sweepUncalibrator; // only using the reading part
  std::array<std::vector<Eigen::Vector3d>, 64> sweep;
  Eigen::Vector3d point;
  double refl;
  while(file >> point(0) >> point(1) >> point(2) >> refl)
  {
    auto probeId = sweepUncalibrator.get_probeId(point);
    assert(probeId < 65);
    sweep.at(probeId).emplace_back(point);
  }
  for(const auto& lasers: sweep) assert(! lasers.empty());
  return sweep;
}


int main(int argc, char* argv[])
{
  using namespace ittik;
  std::ifstream sweepFile(argv[1]);
  std::ofstream outFile(argv[2]);

  std::vector<std::tuple<Eigen::Vector3d, int, double>> points;
  {
    const auto sweep = readSweep(sweepFile);

    // flatten
    for(auto [probeId, lasers]: enumerate(sweep))
      for(auto point: lasers)
        points.emplace_back(point, probeId, atan2(point(1), point(0)));
  }

  // sort based on horizontal angle
  std::sort(points.begin(), points.end(), [](auto a, auto b){return std::get<2>(a) < std::get<2>(b);});

  double stepangle = 2 * M_PI / 4000;

  auto first_pose = liespline::Isometryd3::Identity();
  double first_hori_angle = 0.;
  auto point_error_func = [&stepangle, &first_pose, &first_hori_angle](auto point, auto laserPcloud)
  {
    auto& [point_cloud, probeId, hori_angle] = point;
    auto estimate = first_pose * liespline::expse3((hori_angle - first_hori_angle) * laserPcloud);
    Eigen::Vector3d point_lidar = estimate * point_cloud;
    auto estimate_ = first_pose * liespline::expse3((atan2(point_lidar(1), point_lidar(0)) - first_hori_angle) * laserPcloud);
    Eigen::Vector3d point_lidar_ = estimate_ * point_cloud;
    auto [position, distanceUncor] = unapply_calibration(point_lidar_, kitti_probe_calibration().at(probeId));
    auto hori_error = position - std::lround(position / stepangle) * stepangle;
    auto vert_error = vertical_angle_difference(point_lidar_, kitti_probe_calibration().at(probeId));
    if(fabs(vert_error) > .01)
      hori_error = vert_error = 0.;

    return std::make_pair(hori_error, vert_error);
  };

  // take "middle"
  auto point_at_ref = std::lower_bound(points.begin(), points.end(), 0., [](auto a, auto b){return std::get<2>(a) < b;});
  auto first_point = point_at_ref;
  auto last_point  = point_at_ref;

  int iter = 0;
  // unapply and check if they match with a ground truth measurement
  auto error_func = [&first_point, &point_error_func, &last_point, &iter](auto laserPcloud)
  {
    Eigen::Vector2d error{0., 0.};
    int i = iter;
    int n = 0;
    for(auto pointIt = first_point; pointIt != last_point; ++pointIt)
      if(++i % 10 == 0)
    {
      auto [hori_error, vert_error] = point_error_func(*pointIt, laserPcloud);
      error(0) += hori_error * hori_error;
      error(1) += vert_error * vert_error;
      ++n;
    }
    error(0) = sqrt(error(0) / n);
    error(1) = sqrt(error(1) / n);
    return error;
  };

  // take points until error threshold
  //Eigen::Matrix<double, 6, 1> estimate; estimate << 0.2, 0., 0., 0., 0., 0.; // per radians (.20m*2*pi*10*3.6 = 45km/h)
  Eigen::Matrix<double, 6, 1> estimate; estimate << 0.15, 0., 0., 0., 0., 0.;
  //Eigen::Matrix<double, 6, 1> estimate; estimate << 0.199166, 9.79845e-06, -7.11455e-04, -1.24506e-04, 0.00299908, 3.73258e-04; // one of the runs
  //auto estimate = liespline::Isometryd3::Identity();

  for(int i=1e4;i--;)
  {
    // add points
    std::cout << std::get<0>(*first_point).transpose() << " " << point_error_func(*first_point, estimate).first << " " << point_error_func(*first_point, estimate).second << " "
              << std::get<0>(*last_point).transpose() << " " << point_error_func(*last_point, estimate ).first << " " << point_error_func(*last_point, estimate ).second << " ";
    while(std::get<2>(*first_point) + .6 > std::get<2>(*last_point) && point_error_func(*last_point, estimate).first < stepangle / 3 && fabs(point_error_func(*last_point, estimate).second) < 0.006)
      ++last_point; // .. check if exists...
    auto prev_first_point = first_point;
    // remove points from start
    while(std::get<2>(*first_point) + .3 < std::get<2>(*last_point) && point_error_func(*first_point, estimate).first < stepangle / 3 && fabs(point_error_func(*first_point, estimate).second) < 0.006)
    {
      auto& [point_cloud, probeId, hori_angle] = *first_point;
      auto estimate__ = first_pose * liespline::expse3((hori_angle - first_hori_angle) * estimate);
      Eigen::Vector3d point_lidar_ = estimate__ * point_cloud;
      auto estimate_ = first_pose * liespline::expse3((atan2(point_lidar_(1), point_lidar_(0)) - first_hori_angle) * estimate);
      Eigen::Vector3d point_lidar = estimate_ * point_cloud;
      auto [position, distanceUncor] = unapply_calibration(point_lidar, kitti_probe_calibration().at(probeId));
      outFile << probeId << " " << position << " " << distanceUncor << " "
              << vertical_angle(point_lidar, kitti_probe_calibration().at(probeId)) << " "
              << kitti_probe_calibration().at(probeId).vertCorrection << " "
              << liespline::logse3(estimate_).transpose() << " "
              << hori_angle << std::endl;
      ++first_point;
    }
    first_pose = first_pose * liespline::expse3((std::get<2>(*(first_point - 1)) - first_hori_angle) * estimate);
    first_hori_angle = std::get<2>(*(first_point - 1));

    std::cout << estimate.transpose() << " "
              << error_func(estimate)(0) << " "
              << error_func(estimate)(1) << " "
              << first_point - point_at_ref << " "
              << last_point - first_point << " ";
    std::cout << std::endl;
    ++iter;
    estimate = gradient_descent_step(estimate, error_func);

    { // UUUUUU
      static int derail = 0;
      if(first_point - prev_first_point == 0)
        ++derail;
      else
        derail = 0;
     if(derail == 10) break;
    }
  }

  std::ofstream outFile_("00_odo_unmo_all.txt");
  for(auto& point: points)
  {
    auto& [point_cloud, probeId, hori_angle] = point;
    auto estimate__ = first_pose * liespline::expse3((hori_angle - first_hori_angle) * estimate);
    Eigen::Vector3d point_lidar_ = estimate__ * point_cloud;
    auto estimate_ = first_pose * liespline::expse3((atan2(point_lidar_(1), point_lidar_(0)) - first_hori_angle) * estimate);
    Eigen::Vector3d point_lidar = estimate_ * point_cloud;
    auto [position, distanceUncor] = unapply_calibration(point_lidar, kitti_probe_calibration().at(probeId));
    outFile_ << probeId << " " << position << " " << distanceUncor << " "
            << vertical_angle(point_lidar, kitti_probe_calibration().at(probeId)) << " "
            << kitti_probe_calibration().at(probeId).vertCorrection << " "
            << liespline::logse3(estimate_).transpose() << " "
            << hori_angle << std::endl;
  }
  {
    std::ofstream outFile_("00_odo_unmo_last.txt");
    auto& [point_cloud, probeId, hori_angle] = *last_point;
    auto estimate__ = first_pose * liespline::expse3((hori_angle - first_hori_angle) * estimate);
    Eigen::Vector3d point_lidar_ = estimate__ * point_cloud;
    auto estimate_ = first_pose * liespline::expse3((atan2(point_lidar_(1), point_lidar_(0)) - first_hori_angle) * estimate);
    Eigen::Vector3d point_lidar = estimate_ * point_cloud;
    auto [position, distanceUncor] = unapply_calibration(point_lidar, kitti_probe_calibration().at(probeId));
    outFile_ << probeId << " " << position << " " << distanceUncor << " "
            << vertical_angle(point_lidar, kitti_probe_calibration().at(probeId)) << " "
            << kitti_probe_calibration().at(probeId).vertCorrection << " "
            << liespline::logse3(estimate_).transpose() << " "
            << hori_angle << std::endl;
  }
  {
    std::ofstream outFile_("00_odo_unmo_first.txt");
    auto& [point_cloud, probeId, hori_angle] = *first_point;
    auto estimate__ = first_pose * liespline::expse3((hori_angle - first_hori_angle) * estimate);
    Eigen::Vector3d point_lidar_ = estimate__ * point_cloud;
    auto estimate_ = first_pose * liespline::expse3((atan2(point_lidar_(1), point_lidar_(0)) - first_hori_angle) * estimate);
    Eigen::Vector3d point_lidar = estimate_ * point_cloud;
    auto [position, distanceUncor] = unapply_calibration(point_lidar, kitti_probe_calibration().at(probeId));
    outFile_ << probeId << " " << position << " " << distanceUncor << " "
            << vertical_angle(point_lidar, kitti_probe_calibration().at(probeId)) << " "
            << kitti_probe_calibration().at(probeId).vertCorrection << " "
            << liespline::logse3(estimate_).transpose() << " "
            << hori_angle << std::endl;
  }
  std::cout << first_point - point_at_ref << " "
            << last_point - point_at_ref << std::endl;


  return 0;
}

