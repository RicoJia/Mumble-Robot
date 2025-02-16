#include "rclcpp/rclcpp.hpp"
#include "halo/common/halo_io.hpp"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mumbo_halo_slam");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}