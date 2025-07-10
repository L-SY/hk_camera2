#include "hk_camera/hk_camera_stitching_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <iostream>

HKCameraStitchingNode::HKCameraStitchingNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hk_camera_stitching_node", options) {
    RCLCPP_INFO(get_logger(), "HKCameraStitchingNode constructor called");
    
    // 初始化连续拼接功能
    initialize();
    
    RCLCPP_INFO(get_logger(), "HKCameraStitchingNode constructor completed");
}

void HKCameraStitchingNode::initialize() {
    // 初始化连续拼接特有的功能
    initialize_continuous_stitching();
    setup_publishers();
    
    // 设置参数回调
    auto callback_handle = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            return this->on_param_change(params);
        }
    );
}

void HKCameraStitchingNode::initialize_continuous_stitching() {
    // 声明连续拼接特有的参数
    declare_parameter<bool>("enable_continuous_stitching", true);
    declare_parameter<bool>("debug_mode", false);
    declare_parameter<bool>("deploy_mode", true);
    
    // 特征检测参数
    declare_parameter<int>("orb_features", 1000);
    declare_parameter<double>("match_ratio", 0.75);
    declare_parameter<int>("min_matches", 4);
    
    // 拼接控制参数
    declare_parameter<int>("min_shift", 1);
    declare_parameter<int>("max_shift", 200);
    declare_parameter<int>("max_panorama_width", 10000000);
    declare_parameter<bool>("auto_reset", false);
    declare_parameter<bool>("reset_now", false);
    declare_parameter<bool>("stitch_along_y", true);
    
    // 获取参数值
    enable_continuous_stitching_ = get_parameter("enable_continuous_stitching").as_bool();
    debug_mode_ = get_parameter("debug_mode").as_bool();
    deploy_mode_ = get_parameter("deploy_mode").as_bool();
    
    orb_features_ = get_parameter("orb_features").as_int();
    match_ratio_ = get_parameter("match_ratio").as_double();
    min_matches_ = get_parameter("min_matches").as_int();
    
    min_shift_ = get_parameter("min_shift").as_int();
    max_shift_ = get_parameter("max_shift").as_int();
    max_panorama_width_ = get_parameter("max_panorama_width").as_int();
    auto_reset_ = get_parameter("auto_reset").as_bool();
    reset_now_ = get_parameter("reset_now").as_bool();
    stitch_along_y_ = get_parameter("stitch_along_y").as_bool();
    
    // 初始化ORB检测器
    orb_detector_ = cv::ORB::create(orb_features_);
    
    RCLCPP_INFO(get_logger(), "Continuous stitching parameters initialized");
    RCLCPP_INFO(get_logger(), "Debug mode: %s, Deploy mode: %s", 
                debug_mode_ ? "enabled" : "disabled",
                deploy_mode_ ? "enabled" : "disabled");
}

void HKCameraStitchingNode::setup_publishers() {
    // 设置连续拼接特有的发布者
    
    // 设置连续拼接特有的发布者
    if (enable_continuous_stitching_) {
        continuous_stitched_pub_ = create_publisher<sensor_msgs::msg::Image>("/continuous_stitched_image", 10);
        
        if (debug_mode_ && !deploy_mode_) {
            continuous_debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/continuous_debug_image", 10);
            continuous_matches_pub_ = create_publisher<sensor_msgs::msg::Image>("/continuous_matches_image", 10);
            RCLCPP_INFO(get_logger(), "Debug publishers created");
        }
        
        // 订阅基类的拼接结果
        stitched_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/stitched_image", 10,
            std::bind(&HKCameraStitchingNode::stitched_image_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Continuous stitching publishers and subscriber created");
    }
}

rcl_interfaces::msg::SetParametersResult HKCameraStitchingNode::on_param_change(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    // 处理连续拼接特有的参数
    for (const auto& param : params) {
        if (param.get_name() == "enable_continuous_stitching") {
            enable_continuous_stitching_ = param.as_bool();
            RCLCPP_INFO(get_logger(), "Updated enable_continuous_stitching to: %s", 
                       enable_continuous_stitching_ ? "true" : "false");
        } else if (param.get_name() == "debug_mode") {
            debug_mode_ = param.as_bool();
            RCLCPP_INFO(get_logger(), "Updated debug_mode to: %s", debug_mode_ ? "true" : "false");
        } else if (param.get_name() == "deploy_mode") {
            deploy_mode_ = param.as_bool();
            RCLCPP_INFO(get_logger(), "Updated deploy_mode to: %s", deploy_mode_ ? "true" : "false");
        } else if (param.get_name() == "orb_features") {
            orb_features_ = param.as_int();
            orb_detector_ = cv::ORB::create(orb_features_);
            RCLCPP_INFO(get_logger(), "Updated orb_features to: %d", orb_features_);
        } else if (param.get_name() == "match_ratio") {
            match_ratio_ = param.as_double();
            RCLCPP_INFO(get_logger(), "Updated match_ratio to: %.3f", match_ratio_);
        } else if (param.get_name() == "min_matches") {
            min_matches_ = param.as_int();
            RCLCPP_INFO(get_logger(), "Updated min_matches to: %d", min_matches_);
        } else if (param.get_name() == "min_shift") {
            min_shift_ = param.as_int();
            RCLCPP_INFO(get_logger(), "Updated min_shift to: %d", min_shift_);
        } else if (param.get_name() == "max_shift") {
            max_shift_ = param.as_int();
            RCLCPP_INFO(get_logger(), "Updated max_shift to: %d", max_shift_);
        } else if (param.get_name() == "max_panorama_width") {
            max_panorama_width_ = param.as_int();
            RCLCPP_INFO(get_logger(), "Updated max_panorama_width to: %d", max_panorama_width_);
        } else if (param.get_name() == "auto_reset") {
            auto_reset_ = param.as_bool();
            RCLCPP_INFO(get_logger(), "Updated auto_reset to: %s", auto_reset_ ? "true" : "false");
        } else if (param.get_name() == "reset_now") {
            reset_now_ = param.as_bool();
            if (reset_now_) {
                reset_continuous_panorama();
                RCLCPP_INFO(get_logger(), "Continuous panorama reset");
            }
        } else if (param.get_name() == "stitch_along_y") {
            stitch_along_y_ = param.as_bool();
            RCLCPP_INFO(get_logger(), "Updated stitch_along_y to: %s", stitch_along_y_ ? "true" : "false");
        }
    }
    
    return result;
}

void HKCameraStitchingNode::stitched_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!enable_continuous_stitching_) {
        return;
    }
    
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat current_image = cv_ptr->image;
        
        if (current_image.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty stitched image");
            return;
        }
        
        continuous_stitch_images(current_image, msg->header);
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void HKCameraStitchingNode::continuous_stitch_images(const cv::Mat& current_image, const std_msgs::msg::Header& header) {
    std::lock_guard<std::mutex> lock(panorama_mutex_);
    
    // 初始化全景图或自动重置
    if (auto_reset_ || continuous_panorama_.empty() || last_image_.empty()) {
        RCLCPP_INFO(get_logger(), "Initializing continuous panorama with first frame");
        continuous_panorama_ = current_image.clone();
        last_image_ = current_image.clone();
        publish_continuous_stitched_image(header);
        return;
    }
    
    // 检测和匹配特征
    std::vector<cv::KeyPoint> kp_prev, kp_curr;
    std::vector<cv::DMatch> good_matches;
    
    if (!detect_and_match_features(last_image_, current_image, kp_prev, kp_curr, good_matches)) {
        RCLCPP_WARN(get_logger(), "Feature detection or matching failed");
        last_image_ = current_image.clone();
        return;
    }
    
    // 估计变换
    cv::Point2f offset = estimate_transform(kp_prev, kp_curr, good_matches);
    if (offset.x == 0 && offset.y == 0) {
        RCLCPP_WARN(get_logger(), "Transform estimation failed");
        last_image_ = current_image.clone();
        return;
    }
    
    // 发布匹配可视化（仅在调试模式下）
    if (debug_mode_ && !deploy_mode_ && continuous_matches_pub_) {
        publish_match_visualization(last_image_, current_image, kp_prev, kp_curr, good_matches, header);
    }
    
    // 检查偏移有效性
    float shift = stitch_along_y_ ? offset.y : offset.x;
    if (std::abs(shift) < min_shift_ || std::abs(shift) > max_shift_) {
        RCLCPP_WARN(get_logger(), "Shift %.1f out of valid range [%d, %d]", 
                   shift, min_shift_, max_shift_);
        last_image_ = current_image.clone();
        return;
    }
    
    // 执行连续拼接
    perform_continuous_stitching(current_image, offset);
    
    // 更新最后一帧
    last_image_ = current_image.clone();
    
    // 发布拼接结果
    publish_continuous_stitched_image(header);
}

bool HKCameraStitchingNode::detect_and_match_features(const cv::Mat& prev_image, const cv::Mat& curr_image, 
                                                     std::vector<cv::KeyPoint>& kp_prev, std::vector<cv::KeyPoint>& kp_curr,
                                                     std::vector<cv::DMatch>& good_matches) {
    // 转换为灰度图
    cv::Mat gray_prev, gray_curr;
    cv::cvtColor(prev_image, gray_prev, cv::COLOR_BGR2GRAY);
    cv::cvtColor(curr_image, gray_curr, cv::COLOR_BGR2GRAY);
    
    // 直方图均衡化
    cv::equalizeHist(gray_prev, gray_prev);
    cv::equalizeHist(gray_curr, gray_curr);
    
    // 检测特征点
    cv::Mat desc_prev, desc_curr;
    orb_detector_->detectAndCompute(gray_prev, cv::noArray(), kp_prev, desc_prev);
    orb_detector_->detectAndCompute(gray_curr, cv::noArray(), kp_curr, desc_curr);
    
    if (desc_prev.empty() || desc_curr.empty()) {
        return false;
    }
    
    // 特征匹配
    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(desc_prev, desc_curr, knn_matches, 2);
    
    // Lowe's ratio test
    good_matches.clear();
    for (const auto& match_pair : knn_matches) {
        if (match_pair.size() == 2) {
            const cv::DMatch& m = match_pair[0];
            const cv::DMatch& n = match_pair[1];
            if (m.distance < match_ratio_ * n.distance) {
                good_matches.push_back(m);
            }
        }
    }
    
    return good_matches.size() >= static_cast<size_t>(min_matches_);
}

cv::Point2f HKCameraStitchingNode::estimate_transform(const std::vector<cv::KeyPoint>& kp1, 
                                                     const std::vector<cv::KeyPoint>& kp2, 
                                                     const std::vector<cv::DMatch>& matches) {
    if (matches.size() < static_cast<size_t>(min_matches_)) {
        return cv::Point2f(0, 0);
    }
    
    // 提取匹配点
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& match : matches) {
        pts1.push_back(kp1[match.queryIdx].pt);
        pts2.push_back(kp2[match.trainIdx].pt);
    }
    
    try {
        // 估计仿射变换
        cv::Mat H = cv::estimateAffinePartial2D(pts1, pts2, cv::noArray(), cv::RANSAC, 3.0);
        if (H.empty()) {
            return cv::Point2f(0, 0);
        }
        
        // 提取平移分量
        return cv::Point2f(H.at<double>(0, 2), H.at<double>(1, 2));
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Transform estimation error: %s", e.what());
        return cv::Point2f(0, 0);
    }
}

void HKCameraStitchingNode::perform_continuous_stitching(const cv::Mat& current_image, const cv::Point2f& offset) {
    if (continuous_panorama_.empty()) {
        return;
    }
    
    int shift = static_cast<int>(std::round(stitch_along_y_ ? offset.y : offset.x));
    int add_len = std::abs(shift);
    
    if (add_len == 0) {
        return;
    }
    
    if (!stitch_along_y_) {
        // 水平拼接
        if (shift > 0) {
            cv::Mat strip = current_image(cv::Rect(0, 0, add_len, current_image.rows));
            cv::hconcat(strip, continuous_panorama_, continuous_panorama_);
        } else {
            cv::Mat strip = current_image(cv::Rect(current_image.cols - add_len, 0, add_len, current_image.rows));
            cv::hconcat(continuous_panorama_, strip, continuous_panorama_);
        }
        
        // 限制宽度
        if (continuous_panorama_.cols > max_panorama_width_) {
            int offset = continuous_panorama_.cols - max_panorama_width_;
            continuous_panorama_ = continuous_panorama_(cv::Rect(offset, 0, max_panorama_width_, continuous_panorama_.rows));
        }
    } else {
        // 垂直拼接
        if (continuous_panorama_.cols != current_image.cols) {
            RCLCPP_WARN(get_logger(), "Width mismatch in vertical stitching, resetting");
            continuous_panorama_ = current_image.clone();
            return;
        }
        
        if (shift > 0) {
            cv::Mat strip = current_image(cv::Rect(0, 0, current_image.cols, add_len));
            cv::vconcat(strip, continuous_panorama_, continuous_panorama_);
        } else {
            cv::Mat strip = current_image(cv::Rect(0, current_image.rows - add_len, current_image.cols, add_len));
            cv::vconcat(continuous_panorama_, strip, continuous_panorama_);
        }
    }
    
    RCLCPP_DEBUG(get_logger(), "Continuous stitched with shift: %d, panorama size: %dx%d", 
                shift, continuous_panorama_.cols, continuous_panorama_.rows);
}

void HKCameraStitchingNode::publish_continuous_stitched_image(const std_msgs::msg::Header& header) {
    if (continuous_panorama_.empty() || !continuous_stitched_pub_) {
        return;
    }
    
    cv_bridge::CvImage cv_image;
    cv_image.header = header;
    cv_image.encoding = "bgr8";
    cv_image.image = continuous_panorama_;
    
    continuous_stitched_pub_->publish(*cv_image.toImageMsg());
}

void HKCameraStitchingNode::publish_match_visualization(const cv::Mat& img1, const cv::Mat& img2, 
                                                       const std::vector<cv::KeyPoint>& kp1, const std::vector<cv::KeyPoint>& kp2,
                                                       const std::vector<cv::DMatch>& matches, const std_msgs::msg::Header& header) {
    if (!continuous_matches_pub_) {
        return;
    }
    
    // 限制匹配数量用于可视化
    std::vector<cv::DMatch> matches_to_show(matches.begin(), 
                                           matches.begin() + std::min(static_cast<size_t>(50), matches.size()));
    
    cv::Mat match_img;
    cv::drawMatches(img1, kp1, img2, kp2, matches_to_show, match_img, 
                   cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), 
                   cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    
    cv_bridge::CvImage cv_image;
    cv_image.header = header;
    cv_image.encoding = "bgr8";
    cv_image.image = match_img;
    
    continuous_matches_pub_->publish(*cv_image.toImageMsg());
}

void HKCameraStitchingNode::reset_continuous_panorama() {
    std::lock_guard<std::mutex> lock(panorama_mutex_);
    continuous_panorama_ = cv::Mat();
    last_image_ = cv::Mat();
    RCLCPP_INFO(get_logger(), "Continuous panorama reset");
}

cv::Mat HKCameraStitchingNode::enhance_image(const cv::Mat& image) {
    cv::Mat enhanced;
    // 可以添加图像增强处理，如对比度调整等
    image.copyTo(enhanced);
    return enhanced;
}

cv::Mat HKCameraStitchingNode::apply_histogram_equalization(const cv::Mat& image) {
    cv::Mat result;
    if (image.channels() == 3) {
        cv::cvtColor(image, result, cv::COLOR_BGR2YUV);
        std::vector<cv::Mat> channels;
        cv::split(result, channels);
        cv::equalizeHist(channels[0], channels[0]);
        cv::merge(channels, result);
        cv::cvtColor(result, result, cv::COLOR_YUV2BGR);
    } else {
        cv::equalizeHist(image, result);
    }
    return result;
}

void HKCameraStitchingNode::spin() {
    // 连续拼接节点主循环
    rclcpp::spin(shared_from_this());
}

void HKCameraStitchingNode::process_and_publish_continuous_images() {
    // 这个方法现在通过回调函数处理，不需要在主循环中调用
} 