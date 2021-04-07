#include"io.hpp"
#include"liespline/se3_plot.hpp"

int main(int argc, char* argv[])
{
  using namespace ittik;
  const auto poses = read_odometry_poses(argv[1]);
  std::ofstream file(argv[2]);
  for(auto& pose: poses)
    liespline::plot_se3(pose, file, .05);

  return 0;
}

