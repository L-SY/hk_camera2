#ifndef HK_CAMERA_CAMERA_STITCHING_NODE_HPP
#define HK_CAMERA_CAMERA_STITCHING_NODE_HPP

#include "hk_camera/hk_camera_node.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <atomic>

class HKCameraCameraStitchingNode : public HKCameraNode {
public:
    explicit HKCameraCameraStitchingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    void spin() override;

protected:
    // 重写基类方法
    void setup_publishers() override;
    rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter>& params) override;

private:
    // 拼接特有的功能
    void initialize_stitching_specific();
    bool load_homography_matrix();
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
                                 const cv::Rect& /* left_rect_on_canvas */);
    cv::Mat create_feather_mask(const cv::Mat& mask, int feather_size);
    cv::Mat apply_gamma_correction(const cv::Mat& image, double gamma);
    
    // 图像拼接相关
    cv::Mat homography_matrix_;
    cv::Mat stitched_result_;
    cv::Mat warped_right_;
    cv::Mat debug_result_;
    
    // 平场校正
    bool use_flat_field_;
    cv::Mat flat_left_;
    cv::Mat flat_right_;
    
    // 拼接特有的发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    
    // 拼接参数
    bool enable_stitching_;
    bool publish_individual_images_;
    bool enable_blending_;
    bool publish_individual_roi_;
    double blend_strength_;
    std::string blend_mode_;
    int blend_feather_size_;
    std::string blend_overlap_priority_;
    double blend_priority_strength_;
    double blend_gamma_correction_;
    std::atomic<bool> processing_active_{false};
};

#endif // HK_CAMERA_CAMERA_STITCHING_NODE_HPP 