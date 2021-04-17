#include<iostream>
#include<fstream>
#include"semantics.hpp"

int main(int argc, char* argv[])
{
  using namespace ittik;
  std::ifstream binFile(argv[1], std::ios::in | std::ios::binary);
  std::ofstream txtFile(argv[2]);
  binFile.seekg(0, std::ios::beg);
  while(!binFile.eof())
  {
    uint16_t semantic;
    uint16_t instance;
    binFile.read(reinterpret_cast<char*>(&semantic), sizeof(decltype(semantic)));
    binFile.read(reinterpret_cast<char*>(&instance), sizeof(decltype(instance)));
    if(binFile.good())
    {
      auto rgb = color_map[semantic];
      txtFile << static_cast<int>(std::get<0>(rgb))  
       << " " << static_cast<int>(std::get<1>(rgb)) 
       << " " << static_cast<int>(std::get<2>(rgb)) 
              << std::endl;
    }
  }

  return 0;
}

