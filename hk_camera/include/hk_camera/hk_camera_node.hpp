#pragma once

#include "hk_camera/camera_manager.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

class HKCameraNode : public rclcpp::Node {
public:
  explicit HKCameraNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void spin();

private:
  struct CameraPublisher {
    std::string name;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
  };

  bool load_configs();
  void setup_publishers();
  void setup_dynamic_params();
  rcl_interfaces::msg::SetParametersResult
  on_param_change(const std::vector<rclcpp::Parameter> &params);

  std::vector<CameraParams> configs_;
  CameraManager cam_mgr_;
  std::vector<CameraPublisher> pubs_;
  int loop_rate_hz_ = 30;
  std::vector<CameraParams> runtime_params_; // 用于动态参数
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;
};