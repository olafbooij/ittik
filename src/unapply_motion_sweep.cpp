#include<iostream>
#include<fstream>
#include<Eigen/Core>
#include<iomanip>
#include<chrono>
#include<ctime>

#include<unapply_calibration_sweep.hpp>

// sigh... time is difficult in C++ ... seems no way to read milliseconds... 
struct Time
{
  std::chrono::system_clock::time_point tp;
  float ms;
  Time(auto tp_, auto ms_) : tp(tp_), ms(ms_) {};
};

auto readTimes(auto&& in)
{
  std::vector<Time> times;
  std::tm timetm = {};
  std::string date, ms;
  while(in >> date >> std::get_time(&timetm, "%T") >> ms)
    times.emplace_back(std::chrono::system_clock::from_time_t(std::mktime(&timetm)), stof(ms));
  return times;
}

auto relativeTime(const auto time, const auto timeReference)
{
  return std::chrono::duration_cast<std::chrono::seconds>(time.tp - timeReference.tp).count() + time.ms - timeReference.ms;
}
auto relativeTimes(const auto times, const auto timeReference)
{
  std::vector<double> relTimes;
  for(auto time: times)
    relTimes.emplace_back(relativeTime(time, timeReference));
  return relTimes;
}

int main(int argc, char* argv[])
{
  using namespace std;
  //ifstream sweepFile(argv[1]);
  //ofstream outFile(argv[4]);

  // let's do it hacky:
  // read pose and timing data (oxts + lidar time)
  // approximate time based on heading of point
  // (first just assumed 2pi, later determine the heading-change using pose-date)
  // linearly interpolate pose and unapply

  auto oxtsTimes_ = readTimes(ifstream(argv[2])); // timestamps.txt
  assert(! oxtsTimes_.empty());
  auto oxtsTimes = relativeTimes(oxtsTimes_, oxtsTimes_.front());
  auto laserTimes = relativeTimes(readTimes(ifstream(argv[3])), oxtsTimes_.front());

  for(auto time: laserTimes)
    std::cout << time << std::endl;



  //Eigen::Vector3d point;
  //double refl;
  //while(sweepFile >> point(0) >> point(1) >> point(2) >> refl)
  //{
  //  auto [probeId, position, distanceUncor, vertId_] = sweepUncalibrator(point);
  //  outFile << probeId << " " << position << " " << distanceUncor << " " << vertId_ << std::endl;
  //}

  return 0;
}

