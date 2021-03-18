#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>

#include"io.hpp"
#include<unapply_calibration_sweep.hpp>

#include"liespline/se3_plot.hpp"

int main(int argc, char* argv[])
{
  using namespace std;
  using namespace ittik;
  //ifstream sweepFile(argv[1]);
  ofstream outFile(argv[4]);

  // take two
  // read all lidar data
  // start at pi/2 device position
  // step to future and past as follows
  // predict heading of measurement
  // match actual measurement
  // estimate adjustment of pose given matched measurements
  // perhaps not use oxts data, only to compare pose found 

  // how to estimate adjustment...
  // it is perspective-n-point: N 3d points and I know where they should be measured 
  // one linear step (in some space) would suffice...
  // -> first check if vertical angle is correct in raw data (anyway good to
  // check on that data, there sensor should be standing still.

  ifstream sweepFile("00_odo.txt");
  // read data
  // find pi/2 for all lasers



  return 0;
}

