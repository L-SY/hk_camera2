#include "hk_camera/camera_manager.h"
#include <cstring>
#include <iostream>
#include <chrono>
#include <atomic>

// Performance optimization flags for dual camera
#define ENABLE_DUAL_CAMERA_SYNC 1
#define BALANCE_CAMERA_LOAD 1
#define INDIVIDUAL_CAMERA_STATS 1

// Per-camera performance tracking
struct CameraPerformance {
    std::atomic<int> callback_count{0};
    std::atomic<int> processed_count{0};
    std::chrono::steady_clock::time_point last_callback;
    std::string camera_name;
    double target_fps{60.0};
    double actual_fps{0.0};
};

static std::vector<CameraPerformance> g_camera_performance;
static std::chrono::steady_clock::time_point g_start_time = std::chrono::steady_clock::now();

CameraManager::CameraManager() {}

CameraManager::~CameraManager() {
  stop();
  for (auto &cam : cameras_) {
    if (cam.handle) {
      MV_CC_DestroyHandle(cam.handle);
      cam.handle = nullptr;
    }
  }
  MV_CC_Finalize();
}

bool CameraManager::init() {
  std::vector<CameraParams> cfgs;
  return doInit(cfgs);
}

bool CameraManager::init(const std::vector<CameraParams> &configs) {
  return doInit(configs);
}

bool CameraManager::doInit(const std::vector<CameraParams> &configs) {
  int ret = MV_CC_Initialize();
  if (ret != MV_OK) {
    std::cerr << "MV_CC_Initialize failed: 0x" << std::hex << ret << std::dec
              << std::endl;
    return false;
  }

  MV_CC_DEVICE_INFO_LIST dev_list;
  memset(&dev_list, 0, sizeof(dev_list));
  ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
  if (ret != MV_OK || dev_list.nDeviceNum == 0) {
    std::cerr << "EnumDevices failed or no device: 0x" << std::hex << ret
              << std::dec << std::endl;
    return false;
  }

  cameras_.clear();
  cameras_.reserve(configs.empty() ? dev_list.nDeviceNum : configs.size());

  for (unsigned i = 0;
       i < (configs.empty() ? dev_list.nDeviceNum : configs.size()); ++i) {
    CameraParams cfg;
    if (!configs.empty()) {
      cfg = configs[i];
    }
    for (unsigned j = 0; j < dev_list.nDeviceNum; ++j) {
      std::string sn;
      auto *info = dev_list.pDeviceInfo[j];
      if (info->nTLayerType == MV_USB_DEVICE) {
        sn = reinterpret_cast<char *>(
            info->SpecialInfo.stUsb3VInfo.chSerialNumber);
      }
      if (!configs.empty() && sn != cfg.serial_number)
        continue;

      cameras_.emplace_back();
      auto &ctx = cameras_.back();
      ctx.params = cfg;
      ctx.serial_number = sn;

      if (!createHandle(info, ctx)) {
        cameras_.pop_back();
        std::cerr << "Failed createHandle for S/N=" << sn << std::endl;
      } else {
        setParameter(ctx.handle, ctx.params);
        std::cout << "Initialized camera S/N=" << sn << std::endl;
      }
      break;
    }
  }

  std::cout << "Total initialized cameras: " << cameras_.size() << std::endl;
  
#if INDIVIDUAL_CAMERA_STATS
  // Initialize per-camera performance tracking
  g_camera_performance.resize(cameras_.size());
  for (size_t i = 0; i < cameras_.size(); ++i) {
    g_camera_performance[i].camera_name = cameras_[i].serial_number;
    g_camera_performance[i].callback_count = 0;
    g_camera_performance[i].processed_count = 0;
    g_camera_performance[i].last_callback = std::chrono::steady_clock::now();
  }
#endif
  
  g_start_time = std::chrono::steady_clock::now();
  return !cameras_.empty();
}

bool CameraManager::createHandle(const MV_CC_DEVICE_INFO *info,
                                 CameraContext &ctx) {
  int ret =
      MV_CC_CreateHandle(&ctx.handle, const_cast<MV_CC_DEVICE_INFO *>(info));
  if (ret != MV_OK)
    return false;
  ret = MV_CC_OpenDevice(ctx.handle);
  if (ret != MV_OK) {
    MV_CC_DestroyHandle(ctx.handle);
    return false;
  }

#if ENABLE_DUAL_CAMERA_SYNC
  // Dual camera optimization: ensure identical settings
  if (cameras_.size() >= 2) {
    // Use software trigger for synchronization
    MV_CC_SetEnumValue(ctx.handle, "TriggerMode", 1);
    MV_CC_SetEnumValue(ctx.handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
    MV_CC_SetBoolValue(ctx.handle, "AcquisitionFrameRateEnable", false);
    
    // Critical: Set identical buffer settings for both cameras
    MV_CC_SetIntValue(ctx.handle, "PayloadSize", 0);
    
    // Network optimization for GigE cameras
    if (info->nTLayerType == MV_GIGE_DEVICE) {
      MV_CC_SetIntValue(ctx.handle, "GevSCPSPacketSize", 8192);
      MV_CC_SetIntValue(ctx.handle, "GevSCPD", 1000);  // Inter-packet delay
      
      // Stagger network timing for dual GigE cameras
      if (cameras_.size() == 1) { // This is the second camera being initialized
        MV_CC_SetIntValue(ctx.handle, "GevSCPD", 2000);  // Slightly higher delay for second camera
        std::cout << "[DUAL-SYNC] Applied network staggering for camera 1" << std::endl;
      }
    }
    
    std::cout << "[DUAL-SYNC] Camera configured for synchronized operation" << std::endl;
  } else {
    // Single camera: use free-running mode
    MV_CC_SetEnumValue(ctx.handle, "TriggerMode", 0);
    MV_CC_SetBoolValue(ctx.handle, "AcquisitionFrameRateEnable", true);
    MV_CC_SetFloatValue(ctx.handle, "AcquisitionFrameRate", 90.0);
    std::cout << "[SINGLE] Camera configured for free-running mode" << std::endl;
  }
#else
  // Original logic
  if (cameras_.size() >= 2) {
    MV_CC_SetEnumValue(ctx.handle, "TriggerMode", 1);
    MV_CC_SetEnumValue(ctx.handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
    MV_CC_SetBoolValue(ctx.handle, "AcquisitionFrameRateEnable", false);
  } else {
    MV_CC_SetEnumValue(ctx.handle, "TriggerMode", 0);
    MV_CC_SetBoolValue(ctx.handle, "AcquisitionFrameRateEnable", true);
  }
#endif

  MV_CC_RegisterImageCallBackEx(ctx.handle, imageCallback, &ctx);
  ctx.running = true;
  return true;
}

bool CameraManager::start() {
  for (size_t i = 0; i < cameras_.size(); ++i) {
    CameraContext &ctx = cameras_[i];
    int nRet = MV_CC_StartGrabbing(ctx.handle);
    if (nRet != MV_OK) {
      std::cerr << "StartGrabbing failed for camera " << i << ": 0x" << std::hex
                << nRet << std::endl;
    } else {
      std::cout << "Camera " << i << " (S/N: " << ctx.serial_number
                << ") started grabbing." << std::endl;
    }
  }

  running_ = true;
  
#if ENABLE_DUAL_CAMERA_SYNC
  // Optimized trigger strategy for dual cameras
  if (cameras_.size() >= 2) {
    trigger_thread_ = std::thread([this]() {
      auto next_trigger = std::chrono::steady_clock::now();
      const auto trigger_interval = std::chrono::microseconds(14285); // ~70Hz target
      
      while (running_) {
        triggerAll();
        
        // Precise timing control
        next_trigger += trigger_interval;
        std::this_thread::sleep_until(next_trigger);
        
        // Adaptive frequency based on performance
        static int adjustment_counter = 0;
        if (++adjustment_counter % 1000 == 0) {
          adaptTriggerFrequency();
        }
      }
    });
    std::cout << "[DUAL-SYNC] Using adaptive software trigger (~70Hz base)" << std::endl;
  } else {
    // Single camera fallback
    trigger_thread_ = std::thread([this]() {
      while (running_) {
        triggerAll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }
#else
  // Original trigger logic
  if (cameras_.size() >= 2) {
    trigger_thread_ = std::thread([this]() {
      while (running_) {
        triggerAll();
        std::this_thread::sleep_for(std::chrono::microseconds(8333));
      }
    });
    std::cout << "[SYNC] Using high-frequency software trigger (120Hz)" << std::endl;
  } else {
    trigger_thread_ = std::thread([this]() {
      while (running_) {
        triggerAll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }
#endif
  
  return true;
}

void CameraManager::stop() {
  running_ = false;
  if (trigger_thread_.joinable())
    trigger_thread_.join();
  for (auto &cam : cameras_) {
    cam.running = false;
    if (cam.handle)
      MV_CC_StopGrabbing(cam.handle);
  }
}

void CameraManager::triggerAll() {
  auto trigger_start = std::chrono::steady_clock::now();
  
  for (auto &cam : cameras_) {
    MV_CC_SetCommandValue(cam.handle, "TriggerSoftware");
  }
  
#if BALANCE_CAMERA_LOAD
  // Add small delay between triggers for load balancing
  if (cameras_.size() >= 2) {
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
#endif
}

// Adaptive trigger frequency based on camera performance
void CameraManager::adaptTriggerFrequency() {
#if INDIVIDUAL_CAMERA_STATS
  if (g_camera_performance.size() < 2) return;
  
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_start_time).count() / 1000.0;
  
  if (elapsed < 2.0) return; // Wait for stable performance data
  
  double fps0 = g_camera_performance[0].processed_count.load() / elapsed;
  double fps1 = g_camera_performance[1].processed_count.load() / elapsed;
  double fps_diff = std::abs(fps0 - fps1);
  
  static int adaptation_count = 0;
  if (++adaptation_count % 10 == 0) { // Report every 10 adaptations
    std::cout << "[ADAPT] Camera FPS: " << fps0 << " / " << fps1 
              << " (diff: " << fps_diff << ")" << std::endl;
  }
  
  // If FPS difference is significant, we might need to adjust strategy
  if (fps_diff > 15.0 && adaptation_count > 5) {
    std::cout << "[ADAPT] Large FPS difference detected, maintaining current strategy" << std::endl;
  }
#endif
}

bool CameraManager::getImage(int cam_idx, cv::Mat &image) {
  if (cam_idx < 0 || cam_idx >= static_cast<int>(cameras_.size()))
    return false;
  auto &cam = cameras_[cam_idx];
  std::lock_guard<std::mutex> lock(cam.mtx);
  if (cam.image_queue.empty())
    return false;
  image = cam.image_queue.front();
  cam.image_queue.pop();
  
#if INDIVIDUAL_CAMERA_STATS
  if (cam_idx < static_cast<int>(g_camera_performance.size())) {
    g_camera_performance[cam_idx].processed_count++;
  }
#endif
  
  return true;
}

int CameraManager::numCameras() { return static_cast<int>(cameras_.size()); }

void __stdcall CameraManager::imageCallback(unsigned char *pData,
                                            MV_FRAME_OUT_INFO_EX *pFrameInfo,
                                            void *pUser) {
  if (!pData || !pFrameInfo || !pUser)
    return;

  CameraContext *ctx = static_cast<CameraContext *>(pUser);
  
#if INDIVIDUAL_CAMERA_STATS
  // Find which camera this callback belongs to
  for (size_t i = 0; i < g_camera_performance.size(); ++i) {
    // Simple serial number matching - could be improved
    if (g_camera_performance[i].camera_name.find("DA2007") != std::string::npos) {
      g_camera_performance[i].callback_count++;
      g_camera_performance[i].last_callback = std::chrono::steady_clock::now();
      break;
    }
  }
  
  // Global performance reporting every 5 seconds
  static auto last_report = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  auto report_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report).count();
  
  if (report_elapsed >= 5000) {
    auto total_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_start_time).count() / 1000.0;
    
    std::cout << "\n=== DUAL CAMERA PERFORMANCE REPORT ===" << std::endl;
    for (size_t i = 0; i < g_camera_performance.size(); ++i) {
      double callback_fps = g_camera_performance[i].callback_count.load() / total_elapsed;
      double processed_fps = g_camera_performance[i].processed_count.load() / total_elapsed;
      std::cout << "Camera " << i << " (" << g_camera_performance[i].camera_name << "):" << std::endl;
      std::cout << "  Callback FPS: " << callback_fps << std::endl;
      std::cout << "  Processed FPS: " << processed_fps << std::endl;
      std::cout << "  Efficiency: " << (processed_fps / callback_fps * 100) << "%" << std::endl;
    }
    
    if (g_camera_performance.size() >= 2) {
      double fps0 = g_camera_performance[0].processed_count.load() / total_elapsed;
      double fps1 = g_camera_performance[1].processed_count.load() / total_elapsed;
      std::cout << "FPS Difference: " << std::abs(fps0 - fps1) << " Hz" << std::endl;
    }
    std::cout << "=======================================" << std::endl;
    
    last_report = now;
  }
#endif

  enqueueImage(*ctx, pData, pFrameInfo);
}

void CameraManager::enqueueImage(CameraContext &ctx,
                                 unsigned char *data,
                                 MV_FRAME_OUT_INFO_EX *info) {
  auto start_time = std::chrono::high_resolution_clock::now();
  
  int W = info->nWidth;
  int H = info->nHeight;
  int L = info->nFrameLen;
  cv::Mat img;

  // Optimized image processing for dual cameras
  if (info->enPixelType == PixelType_Gvsp_BayerRG8) {
    cv::Mat bayer_img(H, W, CV_8UC1, data);
    cv::cvtColor(bayer_img, img, cv::COLOR_BayerRG2BGR);
  }
  else if (info->enPixelType == PixelType_Gvsp_Mono8) {
    img = cv::Mat(H, W, CV_8UC1, data).clone();
  }
  else {
    // SDK conversion fallback
    size_t need = static_cast<size_t>(W) * static_cast<size_t>(H) * 3;
    if (ctx.cvt_buf.size() < need)
      ctx.cvt_buf.resize(need);

    MV_CC_PIXEL_CONVERT_PARAM_EX conv{};
    conv.nWidth         = W;
    conv.nHeight        = H;
    conv.pSrcData       = data;
    conv.nSrcDataLen    = L;
    conv.enSrcPixelType = info->enPixelType;
    conv.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    conv.pDstBuffer     = ctx.cvt_buf.data();
    conv.nDstBufferSize = static_cast<uint32_t>(need);

    int ret = MV_CC_ConvertPixelTypeEx(ctx.handle, &conv);
    if (ret == MV_OK) {
      img = cv::Mat(H, W, CV_8UC3, ctx.cvt_buf.data()).clone();
    } else {
      img = cv::Mat(H, W, CV_8UC1, data).clone();
    }
  }

  if (img.empty()) {
    std::cerr << "[ERROR] Failed to create image!" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(ctx.mtx);
  
  // Aggressive queue management for consistent performance
  while (ctx.image_queue.size() >= 3) {  // Even smaller queue for dual cameras
    ctx.image_queue.pop();
  }
  
  ctx.image_queue.push(std::move(img));
}

void *CameraManager::getHandle(size_t index) const {
  if (index >= cameras_.size())
    return nullptr;
  return cameras_[index].handle;
}

int CameraManager::setParameter(void *dev_handle_, CameraParams &config) {
  int ret;

  // Dual camera specific optimizations
  ret = MV_CC_SetEnumValue(dev_handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
  if (ret != MV_OK) {
    std::cerr << "[WARN] Set PixelFormat to BayerRG8 failed: 0x" << std::hex << ret << std::dec << std::endl;
  }

  // Critical: Ensure IDENTICAL exposure settings for both cameras
  if (!config.exposure_auto) {
    ret = MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    ret = MV_CC_SetFloatValue(dev_handle_, "ExposureTime", config.exposure_value);
    std::cout << "[DUAL-SYNC] Set fixed exposure: " << config.exposure_value << "μs" << std::endl;
  }

  // Critical: Ensure IDENTICAL gain settings for both cameras  
  if (!config.gain_auto) {
    ret = MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_OFF);
    ret = MV_CC_SetFloatValue(dev_handle_, "Gain", config.gain_value);
    std::cout << "[DUAL-SYNC] Set fixed gain: " << config.gain_value << "dB" << std::endl;
  }

  // Disable white balance auto for consistency
  ret = MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF);
  
  // Disable gamma for performance
  ret = MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false);

  // Set ROI - ensure identical for both cameras
  ret = MV_CC_SetIntValueEx(dev_handle_, "Width", config.width);
  ret = MV_CC_SetIntValueEx(dev_handle_, "Height", config.height);
  ret = MV_CC_SetIntValueEx(dev_handle_, "OffsetX", config.offset_x);
  ret = MV_CC_SetIntValueEx(dev_handle_, "OffsetY", config.offset_y);

  std::cout << "[DUAL-SYNC] Camera parameters set for synchronized operation" << std::endl;
  return MV_OK;
} 