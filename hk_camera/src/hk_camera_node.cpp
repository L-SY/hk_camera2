#include "hk_camera/hk_camera_node.hpp"
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std::chrono_literals;

HKCameraNode::HKCameraNode(const rclcpp::NodeOptions& options)
    : Node("hk_camera_node", options) {
  declare_parameter<std::string>("config_file", "");
  // 声明所有需要的参数
  declare_parameter<int>("loop_rate_hz", 30);
  declare_parameter<std::string>("camera_name", "cs050");
  declare_parameter<std::string>("serial_number", "");
  declare_parameter<bool>("exposure_auto", true);
  declare_parameter<int>("exposure_min", 15);
  declare_parameter<int>("exposure_max", 13319);
  declare_parameter<int>("exposure_mode", 0);
  declare_parameter<double>("exposure_value", 5000.0);
  declare_parameter<bool>("gain_auto", true);
  declare_parameter<double>("gain_min", 0.0);
  declare_parameter<double>("gain_max", 22.0);
  declare_parameter<double>("gain_value", 6.0);
  declare_parameter<bool>("white_balance_auto", true);
  declare_parameter<int>("roi_width", 2448);
  declare_parameter<int>("roi_height", 2048);
  declare_parameter<int>("roi_offset_x", 0);
  declare_parameter<int>("roi_offset_y", 0);
  
  if (!load_configs()) {
    RCLCPP_FATAL(get_logger(), "Failed to load camera configs");
    return;
  }
  
  if (!cam_mgr_.init(configs_)) {
    RCLCPP_FATAL(get_logger(), "CameraManager init failed");
    return;
  }
  
  cam_mgr_.start();
  setup_publishers();
  setup_dynamic_params();
}

bool HKCameraNode::load_configs() {
  configs_.clear();
  runtime_params_.clear();

  // 读取参数文件路径
  std::string config_file;
  if (!get_parameter_or<std::string>("config_file", config_file, "")) {
    RCLCPP_FATAL(get_logger(), "config_file parameter not set");
    return false;
  }

  YAML::Node config = YAML::LoadFile(config_file);
  if (!config["cameras"]) {
    RCLCPP_FATAL(get_logger(), "No 'cameras' entry in config file");
    return false;
  }

  for (const auto& cam : config["cameras"]) {
    CameraParams cfg;
    cfg.name = cam["name"].as<std::string>("");
    cfg.serial_number = cam["serial_number"].as<std::string>("");
    cfg.exposure_mode = cam["exposure"]["mode"].as<int>(0);
    cfg.exposure_value = static_cast<float>(cam["exposure"]["value"].as<double>(5000.0));
    cfg.exposure_auto = cam["exposure"]["auto"].as<bool>(true);
    cfg.auto_exposure_min = static_cast<int64_t>(cam["exposure"]["min"].as<int>(15));
    cfg.auto_exposure_max = static_cast<int64_t>(cam["exposure"]["max"].as<int>(13319));
    cfg.gain_value = static_cast<float>(cam["gain"]["value"].as<double>(6.0));
    cfg.gain_auto = cam["gain"]["auto"].as<bool>(true);
    cfg.auto_gain_min = static_cast<float>(cam["gain"]["min"].as<double>(0.0));
    cfg.auto_gain_max = static_cast<float>(cam["gain"]["max"].as<double>(22.0));
    cfg.balance_white_auto = cam["white_balance"]["auto"].as<bool>(true);
    cfg.width = static_cast<int64_t>(cam["roi"]["width"].as<int>(2448));
    cfg.height = static_cast<int64_t>(cam["roi"]["height"].as<int>(2048));
    cfg.offset_x = static_cast<int64_t>(cam["roi"]["offset_x"].as<int>(0));
    cfg.offset_y = static_cast<int64_t>(cam["roi"]["offset_y"].as<int>(0));
    cfg.gamma_selector = 0;
    cfg.gamma_value = 1.0f;
    configs_.push_back(cfg);
    runtime_params_.push_back(cfg);
    RCLCPP_INFO(get_logger(), "Loaded camera config: %s (S/N: %s)", cfg.name.c_str(), cfg.serial_number.c_str());
  }

  if (config["loop_rate_hz"]) {
    loop_rate_hz_ = config["loop_rate_hz"].as<int>(30);
  } else {
    loop_rate_hz_ = 30;
  }

  return !configs_.empty();
}

void HKCameraNode::setup_publishers() {
  pubs_.clear();
  for (const auto& cfg : configs_) {
    std::string topic = "/hk_camera/" + cfg.name + "/image_raw";
    pubs_.push_back({cfg.name, this->create_publisher<sensor_msgs::msg::Image>(topic, 10)});
    RCLCPP_INFO(get_logger(), "Advertising on %s", topic.c_str());
  }
}

void HKCameraNode::setup_dynamic_params() {
  // 设置参数重配置回调
  auto callback_handle = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      return this->on_param_change(params);
    }
  );
  
  // 存储callback handle以避免被销毁
  param_callback_handle_ = callback_handle;
  
  RCLCPP_INFO(get_logger(), "Dynamic parameter reconfiguration setup complete");
}

rcl_interfaces::msg::SetParametersResult HKCameraNode::on_param_change(const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto& param : params) {
    RCLCPP_INFO(get_logger(), "Parameter changed: %s", param.get_name().c_str());
    
    // 更新对应的配置参数
    if (!runtime_params_.empty()) {
      auto& cfg = runtime_params_[0]; // 假设单相机配置
      
      if (param.get_name() == "exposure_auto") {
        cfg.exposure_auto = param.as_bool();
      } else if (param.get_name() == "exposure_value") {
        cfg.exposure_value = static_cast<float>(param.as_double());
      } else if (param.get_name() == "exposure_min") {
        cfg.auto_exposure_min = static_cast<int64_t>(param.as_int());
      } else if (param.get_name() == "exposure_max") {
        cfg.auto_exposure_max = static_cast<int64_t>(param.as_int());
      } else if (param.get_name() == "gain_auto") {
        cfg.gain_auto = param.as_bool();
      } else if (param.get_name() == "gain_value") {
        cfg.gain_value = static_cast<float>(param.as_double());
      } else if (param.get_name() == "gain_min") {
        cfg.auto_gain_min = static_cast<float>(param.as_double());
      } else if (param.get_name() == "gain_max") {
        cfg.auto_gain_max = static_cast<float>(param.as_double());
      } else if (param.get_name() == "white_balance_auto") {
        cfg.balance_white_auto = param.as_bool();
      } else if (param.get_name() == "roi_width") {
        cfg.width = static_cast<int64_t>(param.as_int());
      } else if (param.get_name() == "roi_height") {
        cfg.height = static_cast<int64_t>(param.as_int());
      } else if (param.get_name() == "roi_offset_x") {
        cfg.offset_x = static_cast<int64_t>(param.as_int());
      } else if (param.get_name() == "roi_offset_y") {
        cfg.offset_y = static_cast<int64_t>(param.as_int());
      } else if (param.get_name() == "loop_rate_hz") {
        loop_rate_hz_ = param.as_int();
        continue; // 循环频率不需要传递给相机
      }
      
      // 将更新的参数应用到相机
      void* handle = cam_mgr_.getHandle(0);
      if (handle) {
        int ret = cam_mgr_.setParameter(handle, cfg);
        if (ret != 0) { // MV_OK = 0
          RCLCPP_WARN(get_logger(), "Failed to set camera parameter %s: 0x%X", 
                      param.get_name().c_str(), ret);
          result.successful = false;
        } else {
          RCLCPP_INFO(get_logger(), "Successfully updated camera parameter: %s", 
                      param.get_name().c_str());
        }
      } else {
        RCLCPP_WARN(get_logger(), "No camera handle available for parameter update");
        result.successful = false;
      }
    }
  }
  
  return result;
}

void HKCameraNode::spin() {
  rclcpp::Rate rate(loop_rate_hz_);
  while (rclcpp::ok()) {
    for (size_t i = 0; i < static_cast<size_t>(cam_mgr_.numCameras()); ++i) {
      cv::Mat img;
      if (!cam_mgr_.getImage(static_cast<int>(i), img)) continue;
      if (img.empty()) continue;
      
      std_msgs::msg::Header header;
      header.stamp = this->now();
      std::string encoding = (img.channels() == 1) ? "mono8" : "bgr8";
      
      auto msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
      pubs_[i].pub->publish(*msg);
    }
    
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
} 