#include "hk_camera/stitching_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StitchingNode>();
    node->spin();
    rclcpp::shutdown();
    return 0;
} 