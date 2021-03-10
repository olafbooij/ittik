#include<iostream>
#include<fstream>
#include<tuple>
#include<map>

// color map from semantic-kitti-all.yaml (converted bgr to rgb)
using RGB = std::tuple<uint8_t, uint8_t, uint8_t>;
std::map<uint16_t, RGB> color_map
{
 { 0  , {  0,   0,   0}},
 { 1  , {255,   0,   0}},
 { 10 , {100, 150, 245}},
 { 11 , {100, 230, 245}},
 { 13 , {100,  80, 250}},
 { 15 , { 30,  60, 150}},
 { 16 , {  0,   0, 255}},
 { 18 , { 80,  30, 180}},
 { 20 , {  0,   0, 255}},
 { 30 , {255,  30,  30}},
 { 31 , {255,  40, 200}},
 { 32 , {150,  30,  90}},
 { 40 , {255,   0, 255}},
 { 44 , {255, 150, 255}},
 { 48 , { 75,   0,  75}},
 { 49 , {175,   0,  75}},
 { 50 , {255, 200,   0}},
 { 51 , {255, 120,  50}},
 { 52 , {255, 150,   0}},
 { 60 , {150, 255, 170}},
 { 70 , {  0, 175,   0}},
 { 71 , {135,  60,   0}},
 { 72 , {150, 240,  80}},
 { 80 , {255, 240, 150}},
 { 81 , {255,   0,   0}},
 { 99 , { 50, 255, 255}},
 { 252, {100, 150, 245}},
 { 256, {  0,   0, 255}},
 { 253, {255,  40, 200}},
 { 254, {255,  30,  30}},
 { 255, {150,  30,  90}},
 { 257, {100,  80, 250}},
 { 258, { 80,  30, 180}},
 { 259, {  0,   0, 255}},
};

int main(int argc, char* argv[])
{
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
