#pragma once

#include <chrono>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

namespace halo {

// foo.format(CleanFmt) << std::endl;
inline Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

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

class RAIITimer {
  public:
    RAIITimer() { start = std::chrono::high_resolution_clock::now(); }

    ~RAIITimer() {
        auto end                              = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Elapsed time: " << elapsed.count() << "s\n";
    }

  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
};
}   // namespace halo