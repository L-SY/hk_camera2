#ifndef HK_CAMERA_STITCHING_NODE_HPP
#define HK_CAMERA_STITCHING_NODE_HPP

#include "hk_camera/camera_manager.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <string>
#include <atomic>

class HKCameraStitchingNode : public rclcpp::Node {
public:
    explicit HKCameraStitchingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    void spin();

private:
    // 核心功能
    bool load_configs();
    bool load_homography_matrix();
    void setup_publishers();
    void setup_dynamic_params();
    void process_and_publish_images();
    
    // 图像处理
    cv::Mat warp_and_stitch(const cv::Mat& img_left, const cv::Mat& img_right, const cv::Mat& H);
    cv::Mat correct_flat(const cv::Mat& image, const cv::Mat& flat_map, double epsilon = 1e-6);
    cv::Mat apply_roi_crop(const cv::Mat& image, const cv::Rect& roi);
    cv::Rect calculate_roi_from_percent(int image_width, int image_height, 
                                      double x_percent, double y_percent, 
                                      double width_percent, double height_percent);
    
    // 高级图像融合
    cv::Mat blend_images_advanced(const cv::Mat& canvas, const cv::Mat& warped_right, 
                                 const cv::Mat& mask_left, const cv::Mat& mask_right, 
                                 const cv::Rect& left_rect_on_canvas);
    cv::Mat create_feather_mask(const cv::Mat& mask, int feather_size);
    cv::Mat apply_gamma_correction(const cv::Mat& image, double gamma);
    
    // 参数回调
    rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter>& params);
    
    // 相机管理
    CameraManager cam_mgr_;
    std::vector<CameraParams> configs_;
    std::vector<CameraParams> runtime_params_;
    
    // 图像拼接
    cv::Mat homography_matrix_;
    cv::Mat stitched_result_;
    cv::Mat warped_right_;
    cv::Mat debug_result_;
    
    // 平场校正
    bool use_flat_field_;
    cv::Mat flat_left_;
    cv::Mat flat_right_;
    
    // ROS发布者
    struct CameraPublisher {
        std::string name;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    };
    
    std::vector<CameraPublisher> individual_pubs_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    
    // 参数和状态
    bool enable_stitching_;
    bool publish_individual_images_;
    bool enable_blending_;
    bool publish_individual_roi_;
    double blend_strength_;
    std::string blend_mode_;
    int blend_feather_size_;
    std::string blend_overlap_priority_;
    double blend_gamma_correction_;
    int loop_rate_hz_;
    std::atomic<bool> processing_active_{false};
    
    // 动态参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif // HK_CAMERA_STITCHING_NODE_HPP 