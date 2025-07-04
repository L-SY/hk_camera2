#include "hk_camera/hk_camera_node.hpp"
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std::chrono_literals;

HKCameraNode::HKCameraNode(const rclcpp::NodeOptions& options)
    : Node("hk_camera_node", options) {
  declare_parameter<std::string>("config_file", "");
  
  if (!load_configs()) {
    RCLCPP_FATAL(get_logger(), "Failed to load camera configs");
    return;
  }
  
  // 声明全局参数
  declare_parameter<int>("loop_rate_hz", 30);
  
  // 为每个相机声明独立的参数
  for (size_t i = 0; i < configs_.size(); ++i) {
    const auto& cfg = configs_[i];
    std::string prefix = cfg.name + ".";
    
    declare_parameter<bool>(prefix + "exposure_auto", cfg.exposure_auto);
    declare_parameter<int>(prefix + "exposure_min", static_cast<int>(cfg.auto_exposure_min));
    declare_parameter<int>(prefix + "exposure_max", static_cast<int>(cfg.auto_exposure_max));
    declare_parameter<int>(prefix + "exposure_mode", cfg.exposure_mode);
    declare_parameter<double>(prefix + "exposure_value", static_cast<double>(cfg.exposure_value));
    declare_parameter<bool>(prefix + "gain_auto", cfg.gain_auto);
    declare_parameter<double>(prefix + "gain_min", static_cast<double>(cfg.auto_gain_min));
    declare_parameter<double>(prefix + "gain_max", static_cast<double>(cfg.auto_gain_max));
    declare_parameter<double>(prefix + "gain_value", static_cast<double>(cfg.gain_value));
    declare_parameter<bool>(prefix + "white_balance_auto", cfg.balance_white_auto);
    declare_parameter<int>(prefix + "roi_width", static_cast<int>(cfg.width));
    declare_parameter<int>(prefix + "roi_height", static_cast<int>(cfg.height));
    declare_parameter<int>(prefix + "roi_offset_x", static_cast<int>(cfg.offset_x));
    declare_parameter<int>(prefix + "roi_offset_y", static_cast<int>(cfg.offset_y));
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
    
    // 处理全局参数
    if (param.get_name() == "loop_rate_hz") {
      loop_rate_hz_ = param.as_int();
      continue; // 循环频率不需要传递给相机
    }
    
    // 处理相机特定参数
    for (size_t i = 0; i < configs_.size(); ++i) {
      const std::string prefix = configs_[i].name + ".";
      
      if (param.get_name().find(prefix) != 0) continue; // 不是当前相机的参数
      
      auto& cfg = runtime_params_[i];
      std::string param_suffix = param.get_name().substr(prefix.length());
      
      if (param_suffix == "exposure_auto") {
        cfg.exposure_auto = param.as_bool();
      } else if (param_suffix == "exposure_value") {
        cfg.exposure_value = static_cast<float>(param.as_double());
      } else if (param_suffix == "exposure_min") {
        cfg.auto_exposure_min = static_cast<int64_t>(param.as_int());
      } else if (param_suffix == "exposure_max") {
        cfg.auto_exposure_max = static_cast<int64_t>(param.as_int());
      } else if (param_suffix == "gain_auto") {
        cfg.gain_auto = param.as_bool();
      } else if (param_suffix == "gain_value") {
        cfg.gain_value = static_cast<float>(param.as_double());
      } else if (param_suffix == "gain_min") {
        cfg.auto_gain_min = static_cast<float>(param.as_double());
      } else if (param_suffix == "gain_max") {
        cfg.auto_gain_max = static_cast<float>(param.as_double());
      } else if (param_suffix == "white_balance_auto") {
        cfg.balance_white_auto = param.as_bool();
      } else if (param_suffix == "roi_width") {
        cfg.width = static_cast<int64_t>(param.as_int());
      } else if (param_suffix == "roi_height") {
        cfg.height = static_cast<int64_t>(param.as_int());
      } else if (param_suffix == "roi_offset_x") {
        cfg.offset_x = static_cast<int64_t>(param.as_int());
      } else if (param_suffix == "roi_offset_y") {
        cfg.offset_y = static_cast<int64_t>(param.as_int());
      }
      
      // 将更新的参数应用到对应相机
      void* handle = cam_mgr_.getHandle(static_cast<int>(i));
      if (handle) {
        int ret = cam_mgr_.setParameter(handle, cfg);
        if (ret != 0) { // MV_OK = 0
          RCLCPP_WARN(get_logger(), "Failed to set camera %s parameter %s: 0x%X", 
                      configs_[i].name.c_str(), param.get_name().c_str(), ret);
          result.successful = false;
        } else {
          RCLCPP_INFO(get_logger(), "Successfully updated camera %s parameter: %s", 
                      configs_[i].name.c_str(), param.get_name().c_str());
        }
      } else {
        RCLCPP_WARN(get_logger(), "No camera handle available for camera %s", configs_[i].name.c_str());
        result.successful = false;
      }
      break; // 找到对应相机后退出循环
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