// Currently, run this file in the mumble_onboard container because it has the G2O and stuff.
#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp>  // because this is added in cmake 
constexpr const char* first_scan_path = "/home/mumble_robot/data/ch5/first.pcd";
constexpr const char* second_scan_path = "/home/mumble_robot/data/ch5/second.pcd";

TEST(TestKNN, test_bruteforce) {
    halo::TextIO first_scan_io(first_scan_path);
    halo::CloudPtr first(new halo::PointCloudType), second(new halo::PointCloudType);
    pcl::io::loadPCDFile(first_scan_path, *first);
    pcl::io::loadPCDFile(second_scan_path, *second);
    ASSERT_TRUE(true);
}

TEST(TestKNN, AssertionEqual) {
    ASSERT_EQ(1, 1);
}
