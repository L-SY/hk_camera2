#include "hk_camera/hk_camera_stitching_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HKCameraStitchingNode>();
  node->spin();
  rclcpp::shutdown();
  return 0;
} 