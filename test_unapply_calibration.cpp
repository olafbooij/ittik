#include<iostream>
#include<Eigen/Core>

#include<calibration_common.hpp>
#include<apply_calibration.hpp>
#include<unapply_calibration.hpp>
#include<unapply_calibration_iter.hpp>

#include<calibration_kitti.hpp>

int main(/*int argc, char* argv[]*/)
{
  //std::cout << measurementToPoint(2.1, 4.3, example_probe_calibration()).transpose() << std::endl;
  assert((measurementToPoint(2.1, 4.3, example_probe_calibration()) - Eigen::Vector3d(4.59269, -2.57884, -1.00061)).norm() < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(3.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).first - 3.8 ) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(2.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).first - 2.8 ) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(2.8 , 1.3 , example_probe_calibration()), example_probe_calibration()).first - 2.8 ) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(0.8 , 19.3, example_probe_calibration()), example_probe_calibration()).first - 0.8 ) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(3.14, 19.3, example_probe_calibration()), example_probe_calibration()).first - 3.14) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(3.15, 19.3, example_probe_calibration()), example_probe_calibration()).first - 3.15) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(6.15, 19.3, example_probe_calibration()), example_probe_calibration()).first - 6.15) < 1e-5);
  assert(fabs(pointToMeasurement(measurementToPoint(3.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).second - 4.3 ) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(2.8 , 4.3 , example_probe_calibration()), example_probe_calibration()).second - 4.3 ) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(2.8 , 1.3 , example_probe_calibration()), example_probe_calibration()).second - 1.3 ) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(0.8 , 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(3.14, 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(3.15, 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  assert(fabs(pointToMeasurement(measurementToPoint(6.15, 19.3, example_probe_calibration()), example_probe_calibration()).second - 19.3) < 1e-4);
  {
    Eigen::Vector3d point{41.081, 12.512, 1.661};
    auto meas = pointToMeasurement(point, kitti_probe_calibration().at(29));
    std::cout << meas.first << " " << meas.second << std::endl;
    auto point_again = measurementToPoint(meas.first, meas.second, kitti_probe_calibration().at(29));
    std::cout << point.transpose() << std::endl;
    std::cout << point_again.transpose() << std::endl;
    std::cout << (point - point_again).norm() << std::endl;

    {
      pointToMeasurement_iter(point, kitti_probe_calibration().at(29));
    }
  }
  {
    std::cout << std::endl;
    std::pair meas{0.11, 11.5};
    int probe_id = 31;
    std::cout << meas.first << " " << meas.second << std::endl;
    auto point = measurementToPoint(meas.first, meas.second, kitti_probe_calibration().at(probe_id));
    std::cout << point.transpose() << std::endl;
    auto meas_again = pointToMeasurement(point, kitti_probe_calibration().at(probe_id));
    std::cout << meas_again.first << " " << meas_again.second << std::endl;
    auto accudown = [](auto v, int a){ return std::floor(v*a)/a;};
    Eigen::Vector3d point_again{accudown(point(0), 1000), accudown(point(1), 1000), accudown(point(2), 1000)};
    auto meas_again_less = pointToMeasurement(point_again, kitti_probe_calibration().at(probe_id));
    std::cout << meas_again_less.first << " " << meas_again_less.second << std::endl;
  }

  return 0;
}

