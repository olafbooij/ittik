#include<iostream>
#include<vector>

int main(/*int argc, char* argv[]*/)
{
  using namespace std;
  vector v{0.04952,
           0.04642,
           0.04323,
           0.04168,
           0.03861,
           0.03544,
           0.03231,
           0.02920,
           0.02610,
           0.02293,
           0.01984,
           0.01824,
           0.01510,
           0.01200,
           0.00886,
           0.00573,
           0.00265,
           -0.0004,};


  
  auto rot = 45.867/1000/1000;
  for(auto a: v)
    for(auto i = 32;i--;)
      cout << i    << " " << a + rot*i << 
      endl << i+32 << " " << a + rot*i << 
      endl;
  return 0;
}

