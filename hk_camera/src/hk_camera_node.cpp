#include "hk_camera/hk_camera_node.hpp"
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

HKCameraNode::HKCameraNode(const rclcpp::NodeOptions& options)
    : Node("hk_camera_node", options) {
  initialize();
}

HKCameraNode::HKCameraNode(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
  initialize();
}

void HKCameraNode::initialize() {
  // 只在参数未声明时声明
  if (!has_parameter("config_file")) {
    declare_parameter<std::string>("config_file", "");
  }
  
  if (!load_configs()) {
    RCLCPP_FATAL(get_logger(), "Failed to load camera configs");
    return;
  }
  
  // 声明全局参数
  if (!has_parameter("loop_rate_hz")) {
    declare_parameter<int>("loop_rate_hz", 30);
  }
  
  // 为每个相机声明独立的参数
  for (size_t i = 0; i < configs_.size(); ++i) {
    const auto& cfg = configs_[i];
    std::string prefix = cfg.name + ".";
    
    if (!has_parameter(prefix + "exposure_auto")) {
      declare_parameter<bool>(prefix + "exposure_auto", cfg.exposure_auto);
    }
    if (!has_parameter(prefix + "exposure_min")) {
      declare_parameter<int>(prefix + "exposure_min", static_cast<int>(cfg.auto_exposure_min));
    }
    if (!has_parameter(prefix + "exposure_max")) {
      declare_parameter<int>(prefix + "exposure_max", static_cast<int>(cfg.auto_exposure_max));
    }
    if (!has_parameter(prefix + "exposure_mode")) {
      declare_parameter<int>(prefix + "exposure_mode", cfg.exposure_mode);
    }
    if (!has_parameter(prefix + "exposure_value")) {
      declare_parameter<double>(prefix + "exposure_value", static_cast<double>(cfg.exposure_value));
    }
    if (!has_parameter(prefix + "gain_auto")) {
      declare_parameter<bool>(prefix + "gain_auto", cfg.gain_auto);
    }
    if (!has_parameter(prefix + "gain_min")) {
      declare_parameter<double>(prefix + "gain_min", static_cast<double>(cfg.auto_gain_min));
    }
    if (!has_parameter(prefix + "gain_max")) {
      declare_parameter<double>(prefix + "gain_max", static_cast<double>(cfg.auto_gain_max));
    }
    if (!has_parameter(prefix + "gain_value")) {
      declare_parameter<double>(prefix + "gain_value", static_cast<double>(cfg.gain_value));
    }
    if (!has_parameter(prefix + "white_balance_auto")) {
      declare_parameter<bool>(prefix + "white_balance_auto", cfg.balance_white_auto);
    }
    if (!has_parameter(prefix + "roi_width")) {
      declare_parameter<int>(prefix + "roi_width", static_cast<int>(cfg.width));
    }
    if (!has_parameter(prefix + "roi_height")) {
      declare_parameter<int>(prefix + "roi_height", static_cast<int>(cfg.height));
    }
    if (!has_parameter(prefix + "roi_offset_x")) {
      declare_parameter<int>(prefix + "roi_offset_x", static_cast<int>(cfg.offset_x));
    }
    if (!has_parameter(prefix + "roi_offset_y")) {
      declare_parameter<int>(prefix + "roi_offset_y", static_cast<int>(cfg.offset_y));
    }
    if (!has_parameter(prefix + "frame_rate")) {
      declare_parameter<double>(prefix + "frame_rate", cfg.frame_rate);
    }
    if (!has_parameter(prefix + "vignetting_enable")) {
      declare_parameter<bool>(prefix + "vignetting_enable", cfg.vignetting_enable);
    }
    if (!has_parameter(prefix + "vignetting_a")) {
      rcl_interfaces::msg::ParameterDescriptor desc_a;
      desc_a.description = "Vignetting correction parameter A";
      desc_a.floating_point_range.resize(1);
      desc_a.floating_point_range[0].from_value = -2.0;
      desc_a.floating_point_range[0].to_value = 0.0;
      desc_a.floating_point_range[0].step = 0.0;
      declare_parameter<double>(prefix + "vignetting_a", static_cast<double>(cfg.vignetting_a), desc_a);
    }
    if (!has_parameter(prefix + "vignetting_b")) {
      rcl_interfaces::msg::ParameterDescriptor desc_b;
      desc_b.description = "Vignetting correction parameter B";
      desc_b.floating_point_range.resize(1);
      desc_b.floating_point_range[0].from_value = -2.0;
      desc_b.floating_point_range[0].to_value = 0.0;
      desc_b.floating_point_range[0].step = 0.0;
      declare_parameter<double>(prefix + "vignetting_b", static_cast<double>(cfg.vignetting_b), desc_b);
    }
    if (!has_parameter(prefix + "vignetting_c")) {
      rcl_interfaces::msg::ParameterDescriptor desc_c;
      desc_c.description = "Vignetting correction parameter C";
      desc_c.floating_point_range.resize(1);
      desc_c.floating_point_range[0].from_value = -2.0;
      desc_c.floating_point_range[0].to_value = 0.0;
      desc_c.floating_point_range[0].step = 0.0;
      declare_parameter<double>(prefix + "vignetting_c", static_cast<double>(cfg.vignetting_c), desc_c);
    }
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

  // 如果是相对路径，则相对于package的config目录
  if (config_file.front() != '/') {
    try {
      std::string pkg_path = ament_index_cpp::get_package_share_directory("hk_camera");
      config_file = pkg_path + "/config/" + config_file;
      RCLCPP_INFO(get_logger(), "Resolved config file path: %s", config_file.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Cannot find package path for relative config file: %s", e.what());
      return false;
    }
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
    cfg.frame_rate = cam["frame_rate"].as<double>(30.0);
    
    // Load vignetting parameters
    if (cam["vignetting"]) {
      cfg.vignetting_enable = cam["vignetting"]["enable"].as<bool>(false);
      cfg.vignetting_a = static_cast<float>(cam["vignetting"]["a"].as<double>(0.0));
      cfg.vignetting_b = static_cast<float>(cam["vignetting"]["b"].as<double>(0.0));
      cfg.vignetting_c = static_cast<float>(cam["vignetting"]["c"].as<double>(0.0));
    } else {
      cfg.vignetting_enable = false;
      cfg.vignetting_a = 0.0f;
      cfg.vignetting_b = 0.0f;
      cfg.vignetting_c = 0.0f;
    }
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
  
  // Configure QoS for low-latency, high-frequency image streaming
  // Use BEST_EFFORT reliability to avoid blocking on slow subscribers
  // Small queue depth minimizes latency - old frames are dropped
  // Use VOLATILE durability for performance
  rclcpp::QoS qos_profile(5);  // Small queue for low latency (was 100)
  qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
  qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
  
  for (const auto& cfg : configs_) {
    std::string topic = "/hk_camera/" + cfg.name + "/image_raw";
    pubs_.push_back({cfg.name, this->create_publisher<sensor_msgs::msg::Image>(topic, qos_profile)});
    RCLCPP_INFO(get_logger(), "Advertising on %s with BestEffort QoS (depth=5, low-latency)", topic.c_str());
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
  
  // Quick return for empty parameter list
  if (params.empty()) {
    return result;
  }
  
  for (const auto& param : params) {
    try {
      // Skip logging for frequent parameter changes to reduce overhead
      // RCLCPP_DEBUG(get_logger(), "Parameter changed: %s", param.get_name().c_str());
    
      // 处理全局参数
      if (param.get_name() == "loop_rate_hz") {
        loop_rate_hz_ = param.as_int();
        continue; // 循环频率不需要传递给相机
      }
      
      // 处理相机特定参数
      bool param_handled = false;
      for (size_t i = 0; i < configs_.size(); ++i) {
        const std::string prefix = configs_[i].name + ".";
        
        if (param.get_name().find(prefix) != 0) continue; // 不是当前相机的参数
        
        auto& cfg = runtime_params_[i];
        std::string param_suffix = param.get_name().substr(prefix.length());
        
        // Handle vignetting parameters (software-only)
        if (param_suffix == "vignetting_enable") {
          cfg.vignetting_enable = param.as_bool();
          RCLCPP_INFO(get_logger(), "Updated vignetting_enable to: %s", cfg.vignetting_enable ? "true" : "false");
          param_handled = true;
          break;
        } else if (param_suffix == "vignetting_a") {
          double value = param.as_double();
          if (value < -2.0 || value > 0.0) {
            RCLCPP_WARN(get_logger(), "vignetting_a value %.3f out of range [-5.0, 5.0], clamping", value);
            value = std::max(-2.0, std::min(0.0, value));
          }
          cfg.vignetting_a = static_cast<float>(value);
          RCLCPP_INFO(get_logger(), "Updated vignetting_a to: %.3f", cfg.vignetting_a);
          param_handled = true;
        } else if (param_suffix == "vignetting_b") {
          double value = param.as_double();
          if (value < -2.0 || value > 0.0) {
            RCLCPP_WARN(get_logger(), "vignetting_b value %.3f out of range [-5.0, 5.0], clamping", value);
            value = std::max(-2.0, std::min(0.0, value));
          }
          cfg.vignetting_b = static_cast<float>(value);
          RCLCPP_INFO(get_logger(), "Updated vignetting_b to: %.3f", cfg.vignetting_b);
          param_handled = true;
        } else if (param_suffix == "vignetting_c") {
          double value = param.as_double();
          if (value < -2.0 || value > 0.0) {
            RCLCPP_WARN(get_logger(), "vignetting_c value %.3f out of range [-5.0, 5.0], clamping", value);
            value = std::max(-2.0, std::min(0.0, value));
          }
          cfg.vignetting_c = static_cast<float>(value);
          RCLCPP_INFO(get_logger(), "Updated vignetting_c to: %.3f", cfg.vignetting_c);
          param_handled = true;
        } else if (param_suffix == "exposure_auto") {
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
        } else if (param_suffix == "frame_rate") {
          cfg.frame_rate = param.as_double();
        } else {
          continue; // Unknown parameter, skip to next camera
        }
        
        // Apply parameters to camera (both hardware and software parameters)
        RCLCPP_INFO(get_logger(), "Calling updateCameraParams for camera %s with vignetting_enable=%s", 
                    configs_[i].name.c_str(), cfg.vignetting_enable ? "true" : "false");
        bool update_success = cam_mgr_.updateCameraParams(i, cfg);
        if (!update_success) {
          RCLCPP_WARN(get_logger(), "Failed to update camera %s parameters", configs_[i].name.c_str());
          result.successful = false;
        } else {
          RCLCPP_INFO(get_logger(), "Successfully updated camera %s parameters", configs_[i].name.c_str());
        }
        param_handled = true;
        break; // Found matching camera, exit loop
      }
      
      if (!param_handled) {
        RCLCPP_WARN(get_logger(), "Unknown parameter: %s", param.get_name().c_str());
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error processing parameter %s: %s", param.get_name().c_str(), e.what());
      result.successful = false;
      result.reason = "Parameter processing failed: " + std::string(e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown error processing parameter %s", param.get_name().c_str());
      result.successful = false;
      result.reason = "Unknown parameter processing error";
    }
  }
  
  return result;
}

void HKCameraNode::spin() {
  // Performance monitoring variables
  int frame_count = 0;
  int publish_count = 0;
  auto start_time = std::chrono::steady_clock::now();
  auto last_report = start_time;
  
  // Adaptive sleep to maintain target rate without blocking unnecessarily
  const double min_loop_time_us = 1000000.0 / loop_rate_hz_;  // Minimum loop time in microseconds
  
  std::cout << "[INFO] Starting camera node with target rate: " << loop_rate_hz_ << " Hz" << std::endl;
  
  while (rclcpp::ok()) {
    auto loop_start = std::chrono::high_resolution_clock::now();
    
    // Use synchronized image acquisition for multiple cameras
    std::vector<cv::Mat> images;
    bool got_images = false;
    
    if (cam_mgr_.numCameras() > 1) {
      // Multi-camera synchronized acquisition - non-blocking
      got_images = cam_mgr_.getSyncedImages(images);
    } else {
      // Single camera acquisition - non-blocking
      images.resize(1);
      got_images = cam_mgr_.getImage(0, images[0]) && !images[0].empty();
    }
    
    if (got_images) {
      // Use capture timestamp for minimal latency (not current time)
      auto timestamp = this->now();  // Can be optimized further if camera provides timestamps
      
      for (size_t i = 0; i < images.size(); ++i) {
        if (images[i].empty()) continue;
        
        frame_count++;
        std_msgs::msg::Header header;
        header.stamp = timestamp;  // Use same timestamp for all cameras
        std::string encoding = (images[i].channels() == 1) ? "mono8" : "bgr8";
        
        // Optimize cv_bridge conversion - reuse CvImage object if possible
        // Create message directly
        cv_bridge::CvImage cv_image(header, encoding, images[i]);
        auto msg = cv_image.toImageMsg();
        
        // Publish immediately - BestEffort QoS ensures non-blocking
        // Small queue depth (100) with BestEffort will drop old messages if full
        pubs_[i].pub->publish(*msg);
        publish_count++;
      }
      
      // Clear images immediately to free memory
      images.clear();
    }
    
    // Handle ROS callbacks
    rclcpp::spin_some(shared_from_this());
    
    // Adaptive sleep: only sleep if we're running too fast
    auto loop_end = std::chrono::high_resolution_clock::now();
    auto loop_time_us = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();
    
    if (loop_time_us < min_loop_time_us) {
      // We're running faster than target, sleep to maintain rate
      auto sleep_time_us = static_cast<int64_t>(min_loop_time_us - loop_time_us);
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_us));
    }
    // If we're running slower than target, don't sleep - process as fast as possible
    
    // Performance reporting every 5 seconds
    auto now = std::chrono::steady_clock::now();
    auto report_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report).count();
    
    if (report_elapsed >= 5000) {
      double total_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() / 1000.0;
      double frame_fps = frame_count / total_elapsed;
      double publish_fps = publish_count / total_elapsed;
      
      std::cout << "=== ROS NODE PERFORMANCE ===" << std::endl;
      std::cout << "Frame get FPS: " << frame_fps << std::endl;
      std::cout << "Publish FPS: " << publish_fps << std::endl;
      std::cout << "Target rate: " << loop_rate_hz_ << " Hz" << std::endl;
      std::cout << "===========================" << std::endl;
      
      last_report = now;
    }
  }
} 

 