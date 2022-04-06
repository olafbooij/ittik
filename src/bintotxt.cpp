#include<iostream>
#include<fstream>
#include<Eigen/Core>

int main(int argc, char* argv[])
{
  std::ifstream binFile(argv[1], std::ios::in | std::ios::binary);
  std::ofstream txtFile(argv[2]);
  binFile.seekg(0, std::ios::beg);
  while(!binFile.eof())
  {
    Eigen::Vector3f p;
    float refl;
    binFile.read(reinterpret_cast<char*>(&p), 3 * sizeof(float));
    binFile.read(reinterpret_cast<char*>(&refl), sizeof(float));
    if(binFile.good())
      txtFile << p.transpose() << " " << refl << std::endl;
  }

  return 0;
}

