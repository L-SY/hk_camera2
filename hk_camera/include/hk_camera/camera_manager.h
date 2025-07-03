//
// Created by lsy on 25-6-16.
//

#pragma once

#include <atomic>
#include <hk_camera/libMVSapi/MvCameraControl.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <vector>

struct CameraParams {
  std::string name;
  std::string serial_number;

  int exposure_mode;
  float exposure_value;
  int exposure_auto;
  int64_t auto_exposure_min;
  int64_t auto_exposure_max;

  float gain_value;
  int gain_auto;
  float auto_gain_min;
  float auto_gain_max;

  // Gamma Notice cs050 and cs020 is not suppose this config!!!
  int gamma_selector;
  float gamma_value;

  // ROI Notice do not suppose dynamic config!!!
  int64_t width;
  int64_t height;
  int64_t offset_x;
  int64_t offset_y;

  // Color
  int balance_white_auto;
};

class CameraManager {
public:
  CameraManager();
  ~CameraManager();

  bool init();
  bool init(const std::vector<CameraParams> &configs);
  bool start();
  void stop();
  void triggerAll();
  bool getImage(int idx, cv::Mat &image);
  int numCameras();
  void *getHandle(size_t index) const;
  int setParameter(void *dev_handle_, CameraParams &config);

private:
  struct CameraContext {
    void *handle = nullptr;
    std::queue<cv::Mat> image_queue;
    std::atomic<bool> running{false};
    std::mutex mtx;
    std::string serial_number;
    CameraParams params;
    std::vector<uint8_t> cvt_buf;
    CameraContext() = default;
    CameraContext(const CameraContext &) = delete;
    CameraContext &operator=(const CameraContext &) = delete;

    CameraContext(CameraContext &&other) noexcept {
      handle = other.handle;
      image_queue = std::move(other.image_queue);
      running.store(other.running.load());
    }

    CameraContext &operator=(CameraContext &&other) noexcept {
      if (this != &other) {
        handle = other.handle;
        image_queue = std::move(other.image_queue);
        running.store(other.running.load());
      }
      return *this;
    }

    ~CameraContext() = default;
  };

  std::vector<CameraContext> cameras_;
  std::thread trigger_thread_;
  std::atomic<bool> running_{false};

  static void __stdcall imageCallback(unsigned char *pData,
                                      MV_FRAME_OUT_INFO_EX *pFrameInfo,
                                      void *pUser);
  static void enqueueImage(CameraContext &ctx, unsigned char *data,
                           MV_FRAME_OUT_INFO_EX *info);

  bool createHandle(const MV_CC_DEVICE_INFO* info, CameraContext& ctx);

  bool doInit(const std::vector<CameraParams>& configs);
};
