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
  
  // 构造函数重载，允许子类指定节点名称
  explicit HKCameraNode(const std::string& node_name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  
  virtual void spin();

protected:
  struct CameraPublisher {
    std::string name;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
  };

  virtual void initialize();
  virtual bool load_configs();
  virtual void setup_publishers();
  virtual void setup_dynamic_params();
  virtual rcl_interfaces::msg::SetParametersResult
  on_param_change(const std::vector<rclcpp::Parameter> &params);

  std::vector<CameraParams> configs_;
  CameraManager cam_mgr_;
  std::vector<CameraPublisher> pubs_;
  int loop_rate_hz_ = 30;
  std::vector<CameraParams> runtime_params_; // 用于动态参数
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;
};