#include<iostream>
#include<Eigen/Core>

#include<calibration_common.hpp>
#include<apply_calibration.hpp>
#include<unapply_calibration.hpp>
#include<unapply_calibration_iter.hpp>

#include<calibration_kitti.hpp>

int main()
{
  //std::cout << apply_calibration(2.1, 4.3, example_probe_calibration()).transpose() << std::endl;
  assert((apply_calibration(2.1, 4.3, example_probe_calibration()) - Eigen::Vector3d(4.57027, -2.59386, -1.0072)).norm() < 1e-4);
  assert(fabs(unapply_calibration(apply_calibration(3.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).first - 3.8 ) < 1e-5);
  assert(fabs(unapply_calibration(apply_calibration(2.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).first - 2.8 ) < 1e-5);
  assert(fabs(unapply_calibration(apply_calibration(2.8 , 1.3 , example_probe_calibration()), example_probe_calibration()).first - 2.8 ) < 1e-5);
  assert(fabs(unapply_calibration(apply_calibration(0.8 , 19.3, example_probe_calibration()), example_probe_calibration()).first - 0.8 ) < 1e-5);
  assert(fabs(unapply_calibration(apply_calibration(3.14, 19.3, example_probe_calibration()), example_probe_calibration()).first - 3.14) < 1e-5);
  assert(fabs(unapply_calibration(apply_calibration(3.15, 19.3, example_probe_calibration()), example_probe_calibration()).first - 3.15) < 1e-5);
  assert(fabs(unapply_calibration(apply_calibration(6.15, 19.3, example_probe_calibration()), example_probe_calibration()).first - 6.15) < 1e-5);
  assert(fabs(unapply_calibration(apply_calibration(3.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).second - 4.3 ) < 1e-4);
  assert(fabs(unapply_calibration(apply_calibration(2.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).second - 4.3 ) < 1e-4);
  assert(fabs(unapply_calibration(apply_calibration(2.8 , 1.3 , example_probe_calibration()), example_probe_calibration()).second - 1.3 ) < 1e-4);
  assert(fabs(unapply_calibration(apply_calibration(0.8 , 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  assert(fabs(unapply_calibration(apply_calibration(3.14, 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  assert(fabs(unapply_calibration(apply_calibration(3.15, 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  assert(fabs(unapply_calibration(apply_calibration(6.15, 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);


  // ...and some debug stuff. Should rewrite some in "proper" tests.
  if(0)
  {
    Eigen::Vector3d point{41.081, 12.512, 1.661};
    {
      auto [position, distanceUncor] = unapply_calibration(point, kitti_probe_calibration().at(29));
      std::cout << position << " " << distanceUncor << std::endl;
      auto pointAgain = apply_calibration(position, distanceUncor, kitti_probe_calibration().at(29));
      std::cout << point.transpose() << std::endl;
      std::cout << pointAgain.transpose() << std::endl;
      std::cout << (point - pointAgain).norm() << std::endl;
    }
    {
      auto [position, distanceUncor] = unapply_calibration_iter(point, kitti_probe_calibration().at(29));
      std::cout << position << " " << distanceUncor << std::endl;
      auto pointAgain = apply_calibration(position, distanceUncor, kitti_probe_calibration().at(29));
      std::cout << point.transpose() << std::endl;
      std::cout << pointAgain.transpose() << std::endl;
      std::cout << (point - pointAgain).norm() << std::endl;
    }
  }

  if(0)
  {
    std::cout << std::endl;
    std::pair meas{0.11, 11.5};
    int probeId = 31;
    std::cout << meas.first << " " << meas.second << std::endl;
    auto point = apply_calibration(meas.first, meas.second, kitti_probe_calibration().at(probeId));
    std::cout << point.transpose() << std::endl;
    auto measAgain = unapply_calibration(point, kitti_probe_calibration().at(probeId));
    std::cout << measAgain.first << " " << measAgain.second << std::endl;
    auto accudown = [](auto v, int a){ return std::floor(v*a)/a;};
    Eigen::Vector3d pointAgain{accudown(point(0), 1000), accudown(point(1), 1000), accudown(point(2), 1000)};
    auto measAgainLess = unapply_calibration(pointAgain, kitti_probe_calibration().at(probeId));
    std::cout << measAgainLess.first << " " << measAgainLess.second << std::endl;
    auto measAgainLessIter = unapply_calibration_iter(pointAgain, kitti_probe_calibration().at(probeId));
    std::cout << measAgainLessIter.first << " " << measAgainLessIter.second << std::endl;
    assert(fabs(measAgainLessIter.first - meas.first) < 1e-4);
    assert(fabs(measAgainLessIter.second - meas.second) < 1e-2);
  }

  {
    //51 -5.709   0.15 -1.772 0.0532419 0.00909542

    Eigen::Vector3d point{-5.709, 0.15, -1.772};
    std::cout << point.transpose() << std::endl;
    std::cout << std::endl;
    int probeId = 51;
    {
      auto meas = unapply_calibration(point, kitti_probe_calibration().at(probeId));
      std::cout << meas.first << " " << meas.second << std::endl;
      auto pointAgain = apply_calibration(meas.first, meas.second, kitti_probe_calibration().at(probeId));
      std::cout << pointAgain.transpose() << std::endl;
      std::cout << (point - pointAgain).norm()
                << " " << fabs(atan2(point(0), point(1)) - atan2(pointAgain(0), pointAgain(1)))
                << std::endl;
    }
    {
      auto meas = unapply_calibration_iter(point, kitti_probe_calibration().at(probeId));
      std::cout << meas.first << " " << meas.second << std::endl;
      auto pointAgain = apply_calibration(meas.first, meas.second, kitti_probe_calibration().at(probeId));
      std::cout << pointAgain.transpose() << std::endl;
      std::cout << (point - pointAgain).norm()
                << " " << fabs(atan2(point(0), point(1)) - atan2(pointAgain(0), pointAgain(1)))
                << std::endl;
    }
  }
  return 0;
}

