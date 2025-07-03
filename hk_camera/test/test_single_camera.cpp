#include "hk_camera/camera_manager.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    CameraManager cam_mgr;
    if (!cam_mgr.init()) {
        std::cerr << "Failed to initialize camera manager!" << std::endl;
        return 1;
    }
    if (cam_mgr.numCameras() == 0) {
        std::cerr << "No camera found!" << std::endl;
        return 1;
    }
    if (!cam_mgr.start()) {
        std::cerr << "Failed to start camera!" << std::endl;
        return 1;
    }
    // 等待一段时间让相机采集到图像
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    cv::Mat image;
    bool got = false;
    for (int i = 0; i < 10; ++i) {
        if (cam_mgr.getImage(0, image)) {
            got = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!got) {
        std::cerr << "Failed to get image from camera!" << std::endl;
        cam_mgr.stop();
        return 1;
    }
    cv::imwrite("test_output.jpg", image);
    std::cout << "Image saved as test_output.jpg" << std::endl;
    cam_mgr.stop();
    return 0;
} 