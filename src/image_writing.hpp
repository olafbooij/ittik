#pragma once

namespace ittik {

  template<typename ImageT, typename File>  // e.g. Eigen::MatrixXd, std::cout
  void write_pgm(const ImageT& image, File&& file)
  {
    // header
    file << "P2\n" << image.cols() << " " << image.rows() << "\n255\n";
    for(int y = 0; y < image.rows(); ++y)
    {
      for(int x = 0; x < image.cols(); ++x)
        file << std::clamp(static_cast<int>(image(y, x)), 0, 255) << " ";
      file << "\n";
    }
  }

  template<typename ImageT, typename File>  // e.g. Eigen::MatrixXd, std::cout
  void write_ppm(const ImageT& imageR,
                 const ImageT& imageG,
                 const ImageT& imageB, File&& file)
  {
    // header
    file << "P3\n" << imageR.cols() << " " << imageR.rows() << "\n255\n";
    for(int y = 0; y < imageR.rows(); ++y)
    {
      for(int x = 0; x < imageR.cols(); ++x)
      {
        file << std::clamp(static_cast<int>(imageR(y, x)), 0, 255) << " ";
        file << std::clamp(static_cast<int>(imageG(y, x)), 0, 255) << " ";
        file << std::clamp(static_cast<int>(imageB(y, x)), 0, 255) << " ";
      }
      file << "\n";
    }
  }

}
