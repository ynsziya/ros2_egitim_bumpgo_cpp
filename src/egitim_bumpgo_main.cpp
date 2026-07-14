#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ros2_egitim_bumpgo_cpp/EgitimBumpGoNode.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_egitim_bumpgo_cpp::EgitimBumpGoNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}