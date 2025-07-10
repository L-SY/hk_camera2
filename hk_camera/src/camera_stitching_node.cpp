#include "hk_camera/camera_stitching_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>
#include <iostream>

CameraStitchingNode::CameraStitchingNode(const rclcpp::NodeOptions& options)
    : HKCameraNode("camera_stitching_node", options) {
  RCLCPP_INFO(get_logger(), "CameraStitchingNode constructor called");
  // 基类构造函数已经调用了 initialize()，相机已经初始化好了
  // 现在只需要初始化拼接特有的参数和发布者
  initialize_stitching_specific();
  RCLCPP_INFO(get_logger(), "CameraStitchingNode constructor completed");
}

void CameraStitchingNode::initialize_stitching_specific() {
  // 基类已经初始化了相机，现在只需要初始化拼接特有的功能
  
  // 声明拼接特有的参数
  declare_parameter<std::string>("homography_file", "");
  declare_parameter<bool>("enable_stitching", true);
  declare_parameter<bool>("publish_individual_images", false);
  declare_parameter<bool>("use_flat_field", false);
  declare_parameter<std::string>("flat_field_left", "");
  declare_parameter<std::string>("flat_field_right", "");
  
  // 声明融合参数
  declare_parameter<bool>("enable_blending", true);
  declare_parameter<double>("blend_strength", 1.0);
  declare_parameter<std::string>("blend_mode", "linear");
  declare_parameter<int>("blend_feather_size", 50);
  declare_parameter<std::string>("blend_overlap_priority", "left");
  declare_parameter<double>("blend_priority_strength", 0.8);
  declare_parameter<double>("blend_gamma_correction", 1.0);
  declare_parameter<bool>("publish_individual_roi", false);
  
  if (!load_homography_matrix()) {
    RCLCPP_FATAL(get_logger(), "Failed to load homography matrix");
    return;
  }
  
  // 加载平场校正文件（如果启用）
  use_flat_field_ = get_parameter("use_flat_field").as_bool();
  if (use_flat_field_) {
    std::string flat_left_path = get_parameter("flat_field_left").as_string();
    std::string flat_right_path = get_parameter("flat_field_right").as_string();
    
    if (!flat_left_path.empty() && !flat_right_path.empty()) {
      try {
        flat_left_ = cv::imread(flat_left_path, cv::IMREAD_GRAYSCALE);
        flat_right_ = cv::imread(flat_right_path, cv::IMREAD_GRAYSCALE);
        if (flat_left_.empty() || flat_right_.empty()) {
          RCLCPP_WARN(get_logger(), "Failed to load flat field images, disabling flat field correction");
          use_flat_field_ = false;
        } else {
          // 转换为浮点数并归一化
          flat_left_.convertTo(flat_left_, CV_32F, 1.0/255.0);
          flat_right_.convertTo(flat_right_, CV_32F, 1.0/255.0);
          RCLCPP_INFO(get_logger(), "Flat field correction enabled");
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to load flat field files: %s", e.what());
        use_flat_field_ = false;
      }
    }
  }
  
  // 获取其他参数
  enable_stitching_ = get_parameter("enable_stitching").as_bool();
  publish_individual_images_ = get_parameter("publish_individual_images").as_bool();
  enable_blending_ = get_parameter("enable_blending").as_bool();
  publish_individual_roi_ = get_parameter("publish_individual_roi").as_bool();
  blend_strength_ = get_parameter("blend_strength").as_double();
  blend_mode_ = get_parameter("blend_mode").as_string();
  blend_feather_size_ = get_parameter("blend_feather_size").as_int();
  blend_overlap_priority_ = get_parameter("blend_overlap_priority").as_string();
  blend_priority_strength_ = get_parameter("blend_priority_strength").as_double();
  blend_gamma_correction_ = get_parameter("blend_gamma_correction").as_double();
  
  RCLCPP_INFO(get_logger(), "Fusion parameters - mode: %s, strength: %.2f, feather: %d, priority: %s, gamma: %.2f", 
              blend_mode_.c_str(), blend_strength_, blend_feather_size_, blend_overlap_priority_.c_str(), blend_gamma_correction_);
  
  // 声明ROI动态参数（使用百分比，范围0.0-1.0）
  auto roi_desc = rcl_interfaces::msg::ParameterDescriptor{};
  roi_desc.description = "ROI coordinate as percentage (0.0-1.0)";
  roi_desc.floating_point_range.resize(1);
  roi_desc.floating_point_range[0].from_value = 0.0;
  roi_desc.floating_point_range[0].to_value = 1.0;
  
  // 左相机ROI参数（百分比）
  declare_parameter<double>("left_roi_x_percent", 0.0, roi_desc);
  declare_parameter<double>("left_roi_y_percent", 0.0, roi_desc);
  declare_parameter<double>("left_roi_width_percent", 1.0, roi_desc);
  declare_parameter<double>("left_roi_height_percent", 1.0, roi_desc);
  
  // 右相机ROI参数（百分比）
  declare_parameter<double>("right_roi_x_percent", 0.0, roi_desc);
  declare_parameter<double>("right_roi_y_percent", 0.0, roi_desc);
  declare_parameter<double>("right_roi_width_percent", 1.0, roi_desc);
  declare_parameter<double>("right_roi_height_percent", 1.0, roi_desc);
  
  // 拼接后ROI参数（百分比）
  declare_parameter<double>("stitched_roi_x_percent", 0.0, roi_desc);
  declare_parameter<double>("stitched_roi_y_percent", 0.0, roi_desc);
  declare_parameter<double>("stitched_roi_width_percent", 1.0, roi_desc);
  declare_parameter<double>("stitched_roi_height_percent", 1.0, roi_desc);
  
  RCLCPP_INFO(get_logger(), "ROI parameters initialized with percentage values");
  
  RCLCPP_INFO(get_logger(), "Camera stitching node initialized successfully");
  RCLCPP_INFO(get_logger(), "Stitching enabled: %s", enable_stitching_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Publish individual images: %s", publish_individual_images_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Loop rate: %d Hz", loop_rate_hz_);
}



bool CameraStitchingNode::load_homography_matrix() {
  std::string homography_file;
  if (!get_parameter_or<std::string>("homography_file", homography_file, "")) {
    // 尝试默认路径
    try {
      std::string pkg_path = ament_index_cpp::get_package_share_directory("hk_camera");
      homography_file = pkg_path + "/files/H_right_to_left.yaml";
      RCLCPP_INFO(get_logger(), "Using default homography file: %s", homography_file.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Cannot find package path, please specify homography_file parameter");
      return false;
    }
  } else if (homography_file.front() != '/') {
    // 如果是相对路径，则相对于package的files目录
    try {
      std::string pkg_path = ament_index_cpp::get_package_share_directory("hk_camera");
      homography_file = pkg_path + "/files/" + homography_file;
      RCLCPP_INFO(get_logger(), "Resolved homography file: %s", homography_file.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Cannot find package path for relative homography file: %s", e.what());
      return false;
    }
  } else {
    RCLCPP_INFO(get_logger(), "Using absolute homography file: %s", homography_file.c_str());
  }

  try {
    std::string yaml_file = homography_file;
    
    RCLCPP_INFO(get_logger(), "Loading homography from: %s", yaml_file.c_str());
    
    // 如果是 .npy 文件，尝试查找对应的 .yaml 文件
    if (homography_file.substr(homography_file.length() - 4) == ".npy") {
      yaml_file = homography_file.substr(0, homography_file.length() - 4) + ".yaml";
      RCLCPP_INFO(get_logger(), "Converting .npy to .yaml path: %s", yaml_file.c_str());
      if (!std::ifstream(yaml_file).good()) {
        // 如果没有 YAML 文件，创建一个默认的单应矩阵
        RCLCPP_WARN(get_logger(), "Homography YAML file not found, creating identity matrix");
        homography_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        return true;
      }
    }

    cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Cannot open homography file: %s", yaml_file.c_str());
      RCLCPP_ERROR(get_logger(), "File exists: %s", std::ifstream(yaml_file).good() ? "yes" : "no");
      return false;
    }

    // 尝试不同的key名称
    if (fs["homography_matrix"].isNone() == false) {
      fs["homography_matrix"] >> homography_matrix_;
    } else if (fs["homography"].isNone() == false) {
      fs["homography"] >> homography_matrix_;
    } else {
      RCLCPP_ERROR(get_logger(), "No valid homography found in file (tried 'homography_matrix' and 'homography')");
      fs.release();
      return false;
    }
    fs.release();

    if (homography_matrix_.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to load homography matrix from: %s", yaml_file.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Successfully loaded homography matrix from: %s", yaml_file.c_str());
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception loading homography: %s", e.what());
    return false;
  }
}

void CameraStitchingNode::setup_publishers() {
  // 先调用基类的 setup_publishers 来设置个人相机发布者
  HKCameraNode::setup_publishers();
  
  // 设置拼接特有的发布者
  if (enable_stitching_) {
    stitched_pub_ = create_publisher<sensor_msgs::msg::Image>("/stitched_image", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/debug_image", 10);
    RCLCPP_INFO(get_logger(), "Stitching publishers created");
  }
}



rcl_interfaces::msg::SetParametersResult CameraStitchingNode::on_param_change(const std::vector<rclcpp::Parameter>& params) {
  // 先调用基类的参数处理
  rcl_interfaces::msg::SetParametersResult result = HKCameraNode::on_param_change(params);
  
  // 处理拼接特有的参数
  for (const auto& param : params) {
    if (param.get_name() == "enable_stitching") {
      enable_stitching_ = param.as_bool();
      RCLCPP_INFO(get_logger(), "Updated enable_stitching to: %s", enable_stitching_ ? "true" : "false");
    } else if (param.get_name() == "publish_individual_images") {
      publish_individual_images_ = param.as_bool();
      setup_publishers();  // 重新设置发布者
      RCLCPP_INFO(get_logger(), "Updated publish_individual_images to: %s", publish_individual_images_ ? "true" : "false");
    } else if (param.get_name() == "left_roi_x_percent") {
      // 参数已经由ROS2自动更新，这里只需要记录日志
      RCLCPP_INFO(get_logger(), "Updated left_roi_x_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "left_roi_y_percent") {
      RCLCPP_INFO(get_logger(), "Updated left_roi_y_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "left_roi_width_percent") {
      RCLCPP_INFO(get_logger(), "Updated left_roi_width_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "left_roi_height_percent") {
      RCLCPP_INFO(get_logger(), "Updated left_roi_height_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "right_roi_x_percent") {
      RCLCPP_INFO(get_logger(), "Updated right_roi_x_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "right_roi_y_percent") {
      RCLCPP_INFO(get_logger(), "Updated right_roi_y_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "right_roi_width_percent") {
      RCLCPP_INFO(get_logger(), "Updated right_roi_width_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "right_roi_height_percent") {
      RCLCPP_INFO(get_logger(), "Updated right_roi_height_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "stitched_roi_x_percent") {
      RCLCPP_INFO(get_logger(), "Updated stitched_roi_x_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "stitched_roi_y_percent") {
      RCLCPP_INFO(get_logger(), "Updated stitched_roi_y_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "stitched_roi_width_percent") {
      RCLCPP_INFO(get_logger(), "Updated stitched_roi_width_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "stitched_roi_height_percent") {
      RCLCPP_INFO(get_logger(), "Updated stitched_roi_height_percent to: %.3f", param.as_double());
      continue;
    } else if (param.get_name() == "use_flat_field") {
      use_flat_field_ = param.as_bool();
      RCLCPP_INFO(get_logger(), "Updated use_flat_field to: %s", use_flat_field_ ? "true" : "false");
      continue;
    } else if (param.get_name() == "enable_blending") {
      enable_blending_ = param.as_bool();
      RCLCPP_INFO(get_logger(), "Updated enable_blending to: %s", enable_blending_ ? "true" : "false");
      continue;
    } else if (param.get_name() == "blend_strength") {
      blend_strength_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Updated blend_strength to: %.3f", blend_strength_);
      continue;
    } else if (param.get_name() == "blend_mode") {
      blend_mode_ = param.as_string();
      RCLCPP_INFO(get_logger(), "Updated blend_mode to: %s", blend_mode_.c_str());
      continue;
    } else if (param.get_name() == "blend_feather_size") {
      blend_feather_size_ = param.as_int();
      RCLCPP_INFO(get_logger(), "Updated blend_feather_size to: %d", blend_feather_size_);
      continue;
    } else if (param.get_name() == "blend_overlap_priority") {
      blend_overlap_priority_ = param.as_string();
      RCLCPP_INFO(get_logger(), "Updated blend_overlap_priority to: %s", blend_overlap_priority_.c_str());
      continue;
    } else if (param.get_name() == "blend_priority_strength") {
      blend_priority_strength_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Updated blend_priority_strength to: %.3f", blend_priority_strength_);
      continue;
    } else if (param.get_name() == "blend_gamma_correction") {
      blend_gamma_correction_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Updated blend_gamma_correction to: %.3f", blend_gamma_correction_);
      continue;
    } else if (param.get_name() == "publish_individual_roi") {
      publish_individual_roi_ = param.as_bool();
      RCLCPP_INFO(get_logger(), "Updated publish_individual_roi to: %s", publish_individual_roi_ ? "true" : "false");
    }
    // 其他ROI相关参数处理可以在这里添加
  }
  
  return result;
}

cv::Mat CameraStitchingNode::correct_flat(const cv::Mat& image, const cv::Mat& flat_map, double epsilon) {
  cv::Mat image_f32, result;
  image.convertTo(image_f32, CV_32F, 1.0/255.0);
  
  cv::Mat flat_safe;
  cv::max(flat_map, epsilon, flat_safe);
  
  if (image.channels() == 3) {
    std::vector<cv::Mat> channels;
    cv::split(image_f32, channels);
    for (auto& ch : channels) {
      cv::divide(ch, flat_safe, ch);
    }
    cv::merge(channels, image_f32);
  } else {
    cv::divide(image_f32, flat_safe, image_f32);
  }
  
  image_f32.convertTo(result, CV_8U, 255.0);
  return result;
}

cv::Mat CameraStitchingNode::warp_and_stitch(const cv::Mat& img_left, const cv::Mat& img_right, const cv::Mat& H) {
  int h_left = img_left.rows, w_left = img_left.cols;
  int h_right = img_right.rows, w_right = img_right.cols;

  // 计算右图变换后的角点
  std::vector<cv::Point2f> corners_right = {{0, 0}, {static_cast<float>(w_right), 0}, 
                                           {static_cast<float>(w_right), static_cast<float>(h_right)}, 
                                           {0, static_cast<float>(h_right)}};
  std::vector<cv::Point2f> corners_right_trans;
  cv::perspectiveTransform(corners_right, corners_right_trans, H);

  // 计算左图角点
  std::vector<cv::Point2f> corners_left = {{0, 0}, {static_cast<float>(w_left), 0}, 
                                          {static_cast<float>(w_left), static_cast<float>(h_left)}, 
                                          {0, static_cast<float>(h_left)}};

  // 合并所有角点找到边界
  std::vector<cv::Point2f> all_corners = corners_left;
  all_corners.insert(all_corners.end(), corners_right_trans.begin(), corners_right_trans.end());

  cv::Rect rect = cv::boundingRect(all_corners);
  cv::Point2f translation(-rect.x, -rect.y);

  // 创建带平移的变换矩阵
  cv::Mat H_trans = (cv::Mat_<double>(3,3) << 1, 0, translation.x, 0, 1, translation.y, 0, 0, 1) * H;

  // 变换右图
  cv::warpPerspective(img_right, warped_right_, H_trans, rect.size());
  
  // 创建掩码
  cv::Mat mask_right = cv::Mat::zeros(rect.size(), CV_8UC1);
  cv::Mat temp_mask = cv::Mat::ones(img_right.size(), CV_8UC1) * 255;
  cv::warpPerspective(temp_mask, mask_right, H_trans, rect.size());
  
  // 初始化拼接结果为变换后的右图
  stitched_result_ = warped_right_.clone();
  
  // 创建左图掩码
  cv::Rect left_roi(static_cast<int>(translation.x), static_cast<int>(translation.y), w_left, h_left);
  cv::Mat mask_left = cv::Mat::zeros(rect.size(), CV_8UC1);
  mask_left(left_roi) = 255;
  
  // 找到重叠区域
  cv::Mat overlap_mask;
  cv::bitwise_and(mask_left, mask_right, overlap_mask);
  
  // 在非重叠区域直接复制左图
  cv::Mat mask_left_only;
  cv::bitwise_and(mask_left, ~mask_right, mask_left_only);
  img_left.copyTo(stitched_result_(left_roi), mask_left_only(left_roi));
  
  // 在重叠区域进行融合（如果启用）
  if (enable_blending_ && cv::countNonZero(overlap_mask) > 0) {
    // 创建融合权重 - 基于距离左图边缘的距离
    cv::Mat weight_left = cv::Mat::zeros(rect.size(), CV_32F);
    cv::Mat weight_right = cv::Mat::zeros(rect.size(), CV_32F);
    
    // 计算重叠区域的范围
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(overlap_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (!contours.empty()) {
      cv::Rect overlap_rect = cv::boundingRect(contours[0]);
      
      // 在重叠区域创建线性权重
      for (int y = overlap_rect.y; y < overlap_rect.y + overlap_rect.height; y++) {
        for (int x = overlap_rect.x; x < overlap_rect.x + overlap_rect.width; x++) {
          if (overlap_mask.at<uchar>(y, x) > 0) {
            // 计算距离左图右边缘的距离
            int dist_from_left_edge = x - (left_roi.x + left_roi.width);
            int overlap_width = overlap_rect.width;
            
            if (overlap_width > 0) {
              float alpha = std::max(0.0f, std::min(1.0f, float(dist_from_left_edge) / float(overlap_width)));
              weight_right.at<float>(y, x) = alpha;
              weight_left.at<float>(y, x) = 1.0f - alpha;
            } else {
              weight_left.at<float>(y, x) = 0.5f;
              weight_right.at<float>(y, x) = 0.5f;
            }
          }
        }
      }
      
      // 应用融合
      for (int y = overlap_rect.y; y < overlap_rect.y + overlap_rect.height; y++) {
        for (int x = overlap_rect.x; x < overlap_rect.x + overlap_rect.width; x++) {
          if (overlap_mask.at<uchar>(y, x) > 0) {
            // 获取左图在全局坐标中的像素
            int left_x = x - left_roi.x;
            int left_y = y - left_roi.y;
            
            if (left_x >= 0 && left_x < w_left && left_y >= 0 && left_y < h_left) {
              cv::Vec3b left_pixel = img_left.at<cv::Vec3b>(left_y, left_x);
              cv::Vec3b right_pixel = stitched_result_.at<cv::Vec3b>(y, x);
              
              float w_left = weight_left.at<float>(y, x);
              float w_right = weight_right.at<float>(y, x);
              
                             cv::Vec3b blended_pixel;
               for (int c = 0; c < 3; c++) {
                 // 应用融合强度
                 float final_w_left = w_left * blend_strength_ + (1.0f - blend_strength_) * 0.5f;
                 float final_w_right = w_right * blend_strength_ + (1.0f - blend_strength_) * 0.5f;
                 
                 blended_pixel[c] = cv::saturate_cast<uchar>(
                   final_w_left * left_pixel[c] + final_w_right * right_pixel[c]
                 );
               }
              stitched_result_.at<cv::Vec3b>(y, x) = blended_pixel;
            }
          }
        }
             }
     }
   } else if (cv::countNonZero(overlap_mask) > 0) {
     // 如果没有启用融合，在重叠区域直接覆盖（左图优先）
     for (int y = 0; y < left_roi.height; y++) {
       for (int x = 0; x < left_roi.width; x++) {
         int global_x = left_roi.x + x;
         int global_y = left_roi.y + y;
         if (overlap_mask.at<uchar>(global_y, global_x) > 0) {
           stitched_result_.at<cv::Vec3b>(global_y, global_x) = img_left.at<cv::Vec3b>(y, x);
         }
       }
     }
   }

  return stitched_result_;
}

cv::Mat CameraStitchingNode::apply_roi_crop(const cv::Mat& image, const cv::Rect& roi) {
  if (roi.width <= 0 || roi.height <= 0) {
    return image;  // 无效ROI，返回原图
  }
  
  // 确保ROI在图像范围内
  cv::Rect safe_roi = roi & cv::Rect(0, 0, image.cols, image.rows);
  
  if (safe_roi.area() <= 0) {
    RCLCPP_WARN(get_logger(), "ROI is outside image bounds, returning original image");
    return image;
  }
  
  return image(safe_roi).clone();
}

cv::Rect CameraStitchingNode::calculate_roi_from_percent(int image_width, int image_height, 
                                                          double x_percent, double y_percent, 
                                                          double width_percent, double height_percent) {
  int x = static_cast<int>(image_width * x_percent);
  int y = static_cast<int>(image_height * y_percent);
  int width = static_cast<int>(image_width * width_percent);
  int height = static_cast<int>(image_height * height_percent);
  
  // 确保ROI在有效范围内
  x = std::max(0, std::min(x, image_width - 1));
  y = std::max(0, std::min(y, image_height - 1));
  width = std::max(1, std::min(width, image_width - x));
  height = std::max(1, std::min(height, image_height - y));
  
  return cv::Rect(x, y, width, height);
}

void CameraStitchingNode::process_and_publish_images() {
  if (processing_active_.load()) {
    return;  // 避免重入
  }
  
  processing_active_.store(true);
  
  // 获取图像
  std::vector<cv::Mat> images(cam_mgr_.numCameras());
  bool all_images_valid = true;
  
  for (int i = 0; i < cam_mgr_.numCameras(); ++i) {
    if (!cam_mgr_.getImage(i, images[i]) || images[i].empty()) {
      all_images_valid = false;
      break;
    }
  }
  
  if (!all_images_valid) {
    processing_active_.store(false);
    return;
  }
  
  auto current_time = this->now();
  
  // 保存原始图像用于拼接（不进行预裁剪）
  std::vector<cv::Mat> original_images = images;
  
  // 发布单独的相机图像（如果启用）- 可以选择发布原始图像或ROI裁剪后的图像
  if (publish_individual_images_ && pubs_.size() == images.size()) {
    // 决定是否对单独发布的图像进行ROI裁剪
    std::vector<cv::Mat> publish_images = images;
    
    // 根据参数决定是否对单独发布的图像进行ROI裁剪
    bool apply_individual_roi = publish_individual_roi_;
    
    if (apply_individual_roi && images.size() >= 2) {
      // 计算左相机ROI
      cv::Rect left_roi = calculate_roi_from_percent(
        images[0].cols, images[0].rows,
        get_parameter("left_roi_x_percent").as_double(),
        get_parameter("left_roi_y_percent").as_double(),
        get_parameter("left_roi_width_percent").as_double(),
        get_parameter("left_roi_height_percent").as_double()
      );
      
      // 计算右相机ROI
      cv::Rect right_roi = calculate_roi_from_percent(
        images[1].cols, images[1].rows,
        get_parameter("right_roi_x_percent").as_double(),
        get_parameter("right_roi_y_percent").as_double(),
        get_parameter("right_roi_width_percent").as_double(),
        get_parameter("right_roi_height_percent").as_double()
      );
      
      // 应用ROI裁剪到发布图像
      publish_images[0] = apply_roi_crop(images[0], left_roi);
      publish_images[1] = apply_roi_crop(images[1], right_roi);
    }
    
    for (size_t i = 0; i < publish_images.size(); ++i) {
      std_msgs::msg::Header header;
      header.stamp = current_time;
      std::string encoding = (publish_images[i].channels() == 1) ? "mono8" : "bgr8";
      auto msg = cv_bridge::CvImage(header, encoding, publish_images[i]).toImageMsg();
      pubs_[i].pub->publish(*msg);
    }
  }
  
  // 进行图像拼接（如果启用且有至少两个相机）
  if (enable_stitching_ && original_images.size() >= 2 && stitched_pub_ && debug_pub_) {
    try {
      // 第一步：使用原始图像尺寸确定固定画布大小
      cv::Mat img_left_full = original_images[0];
      cv::Mat img_right_full = original_images[1];
      
      // 计算原始图像的画布尺寸（基于完整homography矩阵）
      int h_full = img_left_full.rows, w_full = img_left_full.cols;
      int h_right_full = img_right_full.rows, w_right_full = img_right_full.cols;
      
      std::vector<cv::Point2f> corners_right_full = {{0, 0}, {static_cast<float>(w_right_full), 0}, 
                                                     {static_cast<float>(w_right_full), static_cast<float>(h_right_full)}, 
                                                     {0, static_cast<float>(h_right_full)}};
      std::vector<cv::Point2f> corners_right_trans_full;
      cv::perspectiveTransform(corners_right_full, corners_right_trans_full, homography_matrix_);
      
      std::vector<cv::Point2f> corners_left_full = {{0, 0}, {static_cast<float>(w_full), 0}, 
                                                    {static_cast<float>(w_full), static_cast<float>(h_full)}, 
                                                    {0, static_cast<float>(h_full)}};
      
      std::vector<cv::Point2f> all_corners_full = corners_left_full;
      all_corners_full.insert(all_corners_full.end(), corners_right_trans_full.begin(), corners_right_trans_full.end());
      cv::Rect canvas_rect = cv::boundingRect(all_corners_full);
      cv::Point2f canvas_translation(-canvas_rect.x, -canvas_rect.y);
      
      // 第二步：计算左右相机ROI
      cv::Rect left_roi = calculate_roi_from_percent(
        original_images[0].cols, original_images[0].rows,
        get_parameter("left_roi_x_percent").as_double(),
        get_parameter("left_roi_y_percent").as_double(),
        get_parameter("left_roi_width_percent").as_double(),
        get_parameter("left_roi_height_percent").as_double()
      );
      
      cv::Rect right_roi = calculate_roi_from_percent(
        original_images[1].cols, original_images[1].rows,
        get_parameter("right_roi_x_percent").as_double(),
        get_parameter("right_roi_y_percent").as_double(),
        get_parameter("right_roi_width_percent").as_double(),
        get_parameter("right_roi_height_percent").as_double()
      );
      
      // 第三步：应用ROI裁剪
      cv::Mat img_left = apply_roi_crop(original_images[0], left_roi);
      cv::Mat img_right = apply_roi_crop(original_images[1], right_roi);
      
      // 第四步：在固定画布上进行拼接
      // 创建固定尺寸的画布
      cv::Mat canvas = cv::Mat::zeros(canvas_rect.size(), img_left.type());
      
      // 左图直接放置在画布上（考虑ROI偏移）
      cv::Rect left_rect_on_canvas(
        static_cast<int>(canvas_translation.x + left_roi.x),
        static_cast<int>(canvas_translation.y + left_roi.y),
        img_left.cols,
        img_left.rows
      );
      img_left.copyTo(canvas(left_rect_on_canvas));
      
      // 右图通过homography变换放置在画布上
      cv::Mat H_canvas = (cv::Mat_<double>(3,3) << 1, 0, canvas_translation.x, 0, 1, canvas_translation.y, 0, 0, 1) * homography_matrix_;
      
      // 为右图ROI创建变换矩阵
      cv::Mat T_right_roi = (cv::Mat_<double>(3,3) << 1, 0, right_roi.x, 0, 1, right_roi.y, 0, 0, 1);
      cv::Mat H_right_roi = H_canvas * T_right_roi;
      
      // 变换右图并融合到画布 - 使用白色背景避免黑色三角形
      cv::Mat warped_right;
      cv::warpPerspective(img_right, warped_right, H_right_roi, canvas.size(), 
                         cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
      
      // 创建右图掩码
      cv::Mat mask_right = cv::Mat::zeros(canvas.size(), CV_8UC1);
      cv::Mat temp_mask = cv::Mat::ones(img_right.size(), CV_8UC1) * 255;
      cv::warpPerspective(temp_mask, mask_right, H_right_roi, canvas.size());
      
      // 创建左图掩码
      cv::Mat mask_left = cv::Mat::zeros(canvas.size(), CV_8UC1);
      mask_left(left_rect_on_canvas) = 255;
      
      // 使用改进的融合算法，支持优先级控制
      canvas = blend_images_advanced(canvas, warped_right, mask_left, mask_right, left_rect_on_canvas);
      
      cv::Mat stitched = canvas;
      
      // 平场校正
      if (use_flat_field_) {
        if (!flat_left_.empty() && img_left.size() == flat_left_.size()) {
          img_left = correct_flat(img_left, flat_left_);
        }
        if (!flat_right_.empty() && img_right.size() == flat_right_.size()) {
          img_right = correct_flat(img_right, flat_right_);
        }
      }
      
      // 应用拼接后ROI裁剪
      cv::Rect stitched_roi = calculate_roi_from_percent(
        stitched.cols, stitched.rows,
        get_parameter("stitched_roi_x_percent").as_double(),
        get_parameter("stitched_roi_y_percent").as_double(),
        get_parameter("stitched_roi_width_percent").as_double(),
        get_parameter("stitched_roi_height_percent").as_double()
      );
      
      cv::Mat final_stitched = apply_roi_crop(stitched, stitched_roi);
      
      // 创建调试图像
      debug_result_ = stitched.clone();
      
      // 在调试图像上绘制边界框（基于固定画布和ROI）
      // 绘制左相机ROI边界
      cv::rectangle(debug_result_, left_rect_on_canvas, cv::Scalar(0, 255, 0), 3);
      
      // 绘制右相机变换后的边界
      int h_right = img_right.rows, w_right = img_right.cols;
      std::vector<cv::Point2f> corners_right = {{0, 0}, {static_cast<float>(w_right), 0}, 
                                               {static_cast<float>(w_right), static_cast<float>(h_right)}, 
                                               {0, static_cast<float>(h_right)}};
      std::vector<cv::Point2f> debug_corners_right_trans;
      cv::perspectiveTransform(corners_right, debug_corners_right_trans, H_right_roi);
      
      std::vector<cv::Point> int_corners;
      for (const auto& pt : debug_corners_right_trans) {
        int_corners.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
      }
      cv::polylines(debug_result_, int_corners, true, cv::Scalar(0, 0, 255), 3);
      
      // 绘制拼接后ROI边界
      cv::rectangle(debug_result_, stitched_roi, cv::Scalar(255, 0, 255), 2);
      
      // 发布拼接图像
      std_msgs::msg::Header header;
      header.stamp = current_time;
      auto stitched_msg = cv_bridge::CvImage(header, "bgr8", final_stitched).toImageMsg();
      auto debug_msg = cv_bridge::CvImage(header, "bgr8", debug_result_).toImageMsg();
      
      stitched_pub_->publish(*stitched_msg);
      debug_pub_->publish(*debug_msg);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Stitching error: %s", e.what());
    }
  }
  
  processing_active_.store(false);
}

cv::Mat CameraStitchingNode::blend_images_advanced(const cv::Mat& canvas, const cv::Mat& warped_right, 
                                                   const cv::Mat& mask_left, const cv::Mat& mask_right, 
                                                   const cv::Rect& /* left_rect_on_canvas */) {
  cv::Mat result = canvas.clone();
  
  // 应用伽马校正
  cv::Mat corrected_canvas = result;
  cv::Mat corrected_warped = warped_right;
  if (blend_gamma_correction_ != 1.0) {
    corrected_canvas = apply_gamma_correction(result, blend_gamma_correction_);
    corrected_warped = apply_gamma_correction(warped_right, blend_gamma_correction_);
  }
  
  // 找到重叠区域和非重叠区域
  cv::Mat overlap_mask;
  cv::bitwise_and(mask_left, mask_right, overlap_mask);
  
  // 右图非重叠区域：直接复制右图到结果
  cv::Mat mask_right_only;
  cv::bitwise_and(mask_right, ~mask_left, mask_right_only);
  corrected_warped.copyTo(result, mask_right_only);
  
  // 重叠区域处理 - 根据优先级和融合模式决定
  if (cv::countNonZero(overlap_mask) > 0) {
    
    // 如果禁用融合或使用none模式，根据优先级直接覆盖
    if (!enable_blending_ || blend_mode_ == "none") {
      if (blend_overlap_priority_ == "left") {
        // 左图优先：保持左图（不做任何操作，因为result已经是canvas）
        RCLCPP_DEBUG(get_logger(), "Overlap priority: LEFT (no blending)");
      } else {
        // 右图优先或center：右图覆盖
        corrected_warped.copyTo(result, overlap_mask);
        RCLCPP_DEBUG(get_logger(), "Overlap priority: RIGHT (no blending)");
      }
      return result;
    }
    
    // 有融合的情况下，处理重叠区域
    if (blend_overlap_priority_ == "left") {
      // 左图优先：在重叠区域主要保留左图，少量融合右图
      float left_priority_strength = static_cast<float>(blend_priority_strength_);  // 左图权重可配置
      for (int y = 0; y < result.rows; ++y) {
        for (int x = 0; x < result.cols; ++x) {
          if (overlap_mask.at<uchar>(y, x) > 0) {
            cv::Vec3b left_pixel = corrected_canvas.at<cv::Vec3b>(y, x);
            cv::Vec3b right_pixel = corrected_warped.at<cv::Vec3b>(y, x);
            float final_left_weight = left_priority_strength + (1.0f - left_priority_strength) * (1.0f - blend_strength_);
            float final_right_weight = 1.0f - final_left_weight;
            result.at<cv::Vec3b>(y, x) = left_pixel * final_left_weight + right_pixel * final_right_weight;
          }
        }
      }
      RCLCPP_DEBUG(get_logger(), "Overlap priority: LEFT with blending");
      
    } else if (blend_overlap_priority_ == "right") {
      // 右图优先：在重叠区域主要保留右图，少量融合左图
      float right_priority_strength = static_cast<float>(blend_priority_strength_);  // 右图权重可配置
      for (int y = 0; y < result.rows; ++y) {
        for (int x = 0; x < result.cols; ++x) {
          if (overlap_mask.at<uchar>(y, x) > 0) {
            cv::Vec3b left_pixel = corrected_canvas.at<cv::Vec3b>(y, x);
            cv::Vec3b right_pixel = corrected_warped.at<cv::Vec3b>(y, x);
            float final_right_weight = right_priority_strength + (1.0f - right_priority_strength) * blend_strength_;
            float final_left_weight = 1.0f - final_right_weight;
            result.at<cv::Vec3b>(y, x) = left_pixel * final_left_weight + right_pixel * final_right_weight;
          }
        }
      }
      RCLCPP_DEBUG(get_logger(), "Overlap priority: RIGHT with blending");
      
    } else {
      // center模式：使用标准融合算法
      if (blend_mode_ == "linear") {
        // 线性融合
        for (int y = 0; y < result.rows; ++y) {
          for (int x = 0; x < result.cols; ++x) {
            if (overlap_mask.at<uchar>(y, x) > 0) {
              cv::Vec3b left_pixel = corrected_canvas.at<cv::Vec3b>(y, x);
              cv::Vec3b right_pixel = corrected_warped.at<cv::Vec3b>(y, x);
              result.at<cv::Vec3b>(y, x) = left_pixel * (1.0f - blend_strength_) + right_pixel * blend_strength_;
            }
          }
        }
      } else if (blend_mode_ == "weighted") {
        // 基于距离的加权融合
        cv::Mat dist_left, dist_right;
        cv::distanceTransform(255 - mask_left, dist_left, cv::DIST_L2, 3);
        cv::distanceTransform(255 - mask_right, dist_right, cv::DIST_L2, 3);
        
        for (int y = 0; y < result.rows; ++y) {
          for (int x = 0; x < result.cols; ++x) {
            if (overlap_mask.at<uchar>(y, x) > 0) {
              float d_left = dist_left.at<float>(y, x);
              float d_right = dist_right.at<float>(y, x);
              float total_dist = d_left + d_right;
              
              if (total_dist > 0) {
                float weight_right = (d_left / total_dist) * blend_strength_;
                float weight_left = 1.0f - weight_right;
                
                cv::Vec3b left_pixel = corrected_canvas.at<cv::Vec3b>(y, x);
                cv::Vec3b right_pixel = corrected_warped.at<cv::Vec3b>(y, x);
                result.at<cv::Vec3b>(y, x) = left_pixel * weight_left + right_pixel * weight_right;
              }
            }
          }
        }
      } else if (blend_mode_ == "feather") {
        // 羽化融合
        cv::Mat feather_mask_left = create_feather_mask(mask_left, blend_feather_size_);
        cv::Mat feather_mask_right = create_feather_mask(mask_right, blend_feather_size_);
        
        for (int y = 0; y < result.rows; ++y) {
          for (int x = 0; x < result.cols; ++x) {
            if (overlap_mask.at<uchar>(y, x) > 0) {
              float weight_left = feather_mask_left.at<float>(y, x);
              float weight_right = feather_mask_right.at<float>(y, x);
              float total_weight = weight_left + weight_right;
              
              if (total_weight > 0) {
                weight_left /= total_weight;
                weight_right /= total_weight;
                weight_right *= blend_strength_;
                weight_left = 1.0f - weight_right;
                
                cv::Vec3b left_pixel = corrected_canvas.at<cv::Vec3b>(y, x);
                cv::Vec3b right_pixel = corrected_warped.at<cv::Vec3b>(y, x);
                result.at<cv::Vec3b>(y, x) = left_pixel * weight_left + right_pixel * weight_right;
              }
            }
          }
        }
      }
      RCLCPP_DEBUG(get_logger(), "Overlap priority: CENTER with %s blending", blend_mode_.c_str());
    }
  }
  
  return result;
}

cv::Mat CameraStitchingNode::create_feather_mask(const cv::Mat& mask, int feather_size) {
  cv::Mat feather_mask;
  mask.convertTo(feather_mask, CV_32F, 1.0/255.0);
  
  if (feather_size > 0) {
    cv::Mat dist_transform;
    cv::distanceTransform(mask, dist_transform, cv::DIST_L2, 3);
    
    // 创建羽化效果
    for (int y = 0; y < feather_mask.rows; ++y) {
      for (int x = 0; x < feather_mask.cols; ++x) {
        if (mask.at<uchar>(y, x) > 0) {
          float dist = dist_transform.at<float>(y, x);
          if (dist < feather_size) {
            feather_mask.at<float>(y, x) = dist / feather_size;
          } else {
            feather_mask.at<float>(y, x) = 1.0f;
          }
        } else {
          feather_mask.at<float>(y, x) = 0.0f;
        }
      }
    }
  }
  
  return feather_mask;
}

cv::Mat CameraStitchingNode::apply_gamma_correction(const cv::Mat& image, double gamma) {
  cv::Mat result;
  cv::Mat lookup_table(1, 256, CV_8U);
  uchar* p = lookup_table.ptr();
  for (int i = 0; i < 256; ++i) {
    p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
  }
  cv::LUT(image, lookup_table, result);
  return result;
}

void CameraStitchingNode::spin() {
  rclcpp::Rate rate(loop_rate_hz_);
  
  // 性能监控变量
  int frame_count = 0;
  int stitch_count = 0;
  auto start_time = std::chrono::steady_clock::now();
  auto last_report = start_time;
  
  RCLCPP_INFO(get_logger(), "Starting camera stitching node with loop rate: %d Hz", loop_rate_hz_);
  
  while (rclcpp::ok()) {
    auto loop_start = std::chrono::high_resolution_clock::now();
    
    // 处理和发布图像
    process_and_publish_images();
    frame_count++;
    if (enable_stitching_) stitch_count++;
    
    // 性能报告
    auto now = std::chrono::steady_clock::now();
    auto report_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report).count();
    
    if (report_elapsed >= 5000) {  // 每5秒报告一次
      double total_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() / 1000.0;
      double frame_fps = frame_count / total_elapsed;
      double stitch_fps = stitch_count / total_elapsed;
      
      RCLCPP_INFO(get_logger(), "=== STITCHING NODE PERFORMANCE ===");
      RCLCPP_INFO(get_logger(), "Frame processing FPS: %.2f", frame_fps);
      if (enable_stitching_) {
        RCLCPP_INFO(get_logger(), "Stitching FPS: %.2f", stitch_fps);
      }
      RCLCPP_INFO(get_logger(), "Loop rate setting: %d Hz", loop_rate_hz_);
      RCLCPP_INFO(get_logger(), "==================================");
      
      last_report = now;
    }
    
    rclcpp::spin_some(shared_from_this());
    
    auto loop_end = std::chrono::high_resolution_clock::now();
    auto loop_time = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
    
    // 记录长循环时间
    if (loop_time.count() > 50000) {  // > 50ms
      RCLCPP_WARN(get_logger(), "Long loop time: %ld μs", loop_time.count());
    }
    
    rate.sleep();
  }
} 