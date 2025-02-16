#pragma once

#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

namespace halo {
class TextIO {
public:
  TextIO(const std::string &file_path) : fin(file_path) {
    if (!fin.is_open()) {
      throw std::runtime_error("Failed to open file: " + file_path);
    } else {
      std::cout << "File opened successfully: " << file_path << std::endl;
    }
  }

private:
  std::ifstream fin;
};
} // namespace halo