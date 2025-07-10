#ifndef HK_CAMERA_STITCHING_NODE_HPP
#define HK_CAMERA_STITCHING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>

class HKCameraStitchingNode : public rclcpp::Node {
public:
    explicit HKCameraStitchingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    void spin();

protected:
    // 连续拼接节点方法
    void initialize();
    void setup_publishers();
    rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter>& params);

private:
    // 连续拼接特有的功能
    void initialize_continuous_stitching();
    void process_and_publish_continuous_images();
    
    // 连续拼接核心功能 - 订阅基类的拼接结果进行连续拼接
    void stitched_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void continuous_stitch_images(const cv::Mat& current_image, const std_msgs::msg::Header& header);
    bool detect_and_match_features(const cv::Mat& prev_image, const cv::Mat& curr_image, 
                                  std::vector<cv::KeyPoint>& kp_prev, std::vector<cv::KeyPoint>& kp_curr,
                                  std::vector<cv::DMatch>& good_matches);
    cv::Point2f estimate_transform(const std::vector<cv::KeyPoint>& kp1, const std::vector<cv::KeyPoint>& kp2, 
                                  const std::vector<cv::DMatch>& matches);
    void perform_continuous_stitching(const cv::Mat& current_image, const cv::Point2f& offset);
    void publish_continuous_stitched_image(const std_msgs::msg::Header& header);
    void publish_match_visualization(const cv::Mat& img1, const cv::Mat& img2, 
                                   const std::vector<cv::KeyPoint>& kp1, const std::vector<cv::KeyPoint>& kp2,
                                   const std::vector<cv::DMatch>& matches, const std_msgs::msg::Header& header);
    void reset_continuous_panorama();
    
    // 图像处理辅助函数
    cv::Mat enhance_image(const cv::Mat& image);
    cv::Mat apply_histogram_equalization(const cv::Mat& image);
    
    // 连续拼接相关变量
    cv::Mat continuous_panorama_;
    cv::Mat last_image_;
    std::mutex panorama_mutex_;
    
    // ORB特征检测器
    cv::Ptr<cv::ORB> orb_detector_;
    
    // 订阅基类的拼接结果
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stitched_image_sub_;
    
    // 连续拼接发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr continuous_stitched_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr continuous_debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr continuous_matches_pub_;
    
    // 连续拼接参数
    bool enable_continuous_stitching_;
    bool debug_mode_;
    bool deploy_mode_;
    
    // 特征匹配参数
    int orb_features_;
    double match_ratio_;
    int min_matches_;
    
    // 拼接控制参数
    int min_shift_;
    int max_shift_;
    int max_panorama_width_;
    bool auto_reset_;
    bool reset_now_;
    bool stitch_along_y_;
    
    std::atomic<bool> continuous_processing_active_{false};
};

#endif // HK_CAMERA_STITCHING_NODE_HPP 