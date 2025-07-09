#include "hk_camera/camera_manager.h"
#include <cstring>
#include <iostream>
#include <chrono>
#include <atomic>

// Global counters for performance analysis
static std::atomic<int> g_callback_count{0};
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
  // Reset callback statistics
  g_callback_count = 0;
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

  if (cameras_.size() >= 2) {
    MV_CC_SetEnumValue(ctx.handle, "TriggerMode", 1);
    MV_CC_SetEnumValue(ctx.handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
    MV_CC_SetBoolValue(ctx.handle, "AcquisitionFrameRateEnable", false);
  } else {
    MV_CC_SetEnumValue(ctx.handle, "TriggerMode", 0);
    MV_CC_SetBoolValue(ctx.handle, "AcquisitionFrameRateEnable", true);
  }

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
  
  // Use average frame rate from camera configs for trigger frequency
  double avg_frame_rate = getAverageFrameRate();
  int trigger_interval_us = static_cast<int>(1000000.0 / avg_frame_rate);
  
  std::cout << "[SYNC] Using frame rate: " << avg_frame_rate << " Hz (interval: " << trigger_interval_us << " μs)" << std::endl;
  
  if (cameras_.size() >= 2) {
    trigger_thread_ = std::thread([this, trigger_interval_us]() {
      while (running_) {
        triggerAll();
        std::this_thread::sleep_for(std::chrono::microseconds(trigger_interval_us));
      }
    });
    std::cout << "[SYNC] Using synchronized software trigger" << std::endl;
  } else {
    // For single camera, still use trigger for consistency
    trigger_thread_ = std::thread([this, trigger_interval_us]() {
      while (running_) {
        triggerAll();
        std::this_thread::sleep_for(std::chrono::microseconds(trigger_interval_us));
      }
    });
  }
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
  for (auto &cam : cameras_) {
    MV_CC_SetCommandValue(cam.handle, "TriggerSoftware");
  }
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
  return true;
}

int CameraManager::numCameras() { return static_cast<int>(cameras_.size()); }

void __stdcall CameraManager::imageCallback(unsigned char *pData,
                                            MV_FRAME_OUT_INFO_EX *pFrameInfo,
                                            void *pUser) {
  if (!pData || !pFrameInfo || !pUser)
    return;

  CameraContext *ctx = static_cast<CameraContext *>(pUser);
  
  // Increment frame counter for synchronization
  ctx->frame_counter.fetch_add(1);
  
  // Performance tracking
  g_callback_count++;
  static int last_count = 0;
  static auto last_time = std::chrono::steady_clock::now();
  
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
  
  if (elapsed >= 5000) {  // Report every 5 seconds
    int current_count = g_callback_count.load();
    double fps = (current_count - last_count) * 1000.0 / elapsed;
    std::cout << "[CALLBACK] FPS: " << fps << " (total: " << current_count << ")" << std::endl;
    last_count = current_count;
    last_time = now;
  }

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

  // 处理8位Bayer格式
  if (info->enPixelType == PixelType_Gvsp_BayerRG8) {
    cv::Mat bayer_img(H, W, CV_8UC1, data);
    cv::cvtColor(bayer_img, img, cv::COLOR_BayerBG2BGR);  // 8位BayerRG也用BG模式修正颜色
  }
  else if (info->enPixelType == PixelType_Gvsp_Mono8) {
    // 单通道灰度图，直接使用
    img = cv::Mat(H, W, CV_8UC1, data).clone();
  }
  else {
    // 尝试海康SDK的BGR转换作为备选方案
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
      std::cerr << "[WARN] SDK conversion failed: 0x" << std::hex << ret << std::dec << std::endl;
      // 最后的降级方案：直接作为灰度图使用
      img = cv::Mat(H, W, CV_8UC1, data).clone();
    }
  }

  if (img.empty()) {
    std::cerr << "[ERROR] Failed to create image!" << std::endl;
    return;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  
  // Log processing time occasionally
  static int process_count = 0;
  process_count++;
  if (process_count % 100 == 0) {
    std::cout << "[PROCESS] Image processing took: " << duration.count() << " μs" << std::endl;
  }

  std::lock_guard<std::mutex> lock(ctx.mtx);
  
  // Optimization: Prevent queue overflow that can cause memory issues and latency
  while (ctx.image_queue.size() >= 5) {  // Limit queue size to 5
    ctx.image_queue.pop();  // Drop oldest frame
  }
  
  ctx.image_queue.push(img.clone());
  
  // Warn if queue is getting large
  if (ctx.image_queue.size() > 3) {
    static int warn_count = 0;
    if (++warn_count % 100 == 0) {  // Reduce log spam
      std::cout << "[WARN] Image queue size: " << ctx.image_queue.size() << std::endl;
    }
  }
}

bool CameraManager::getSyncedImages(std::vector<cv::Mat> &images) {
  if (cameras_.size() <= 1) {
    // For single camera, just get the image normally
    images.resize(1);
    return getImage(0, images[0]);
  }
  
  // For multiple cameras, wait for frame synchronization
  uint64_t target_frame = sync_frame_counter_.load() + 1;
  
  // Wait for all cameras to reach the target frame
  if (!waitForFrameSync(target_frame)) {
    return false;
  }
  
  // All cameras have reached the target frame, get images
  images.resize(cameras_.size());
  bool success = true;
  
  for (size_t i = 0; i < cameras_.size(); ++i) {
    std::lock_guard<std::mutex> lock(cameras_[i].mtx);
    if (cameras_[i].image_queue.empty()) {
      success = false;
      break;
    }
    images[i] = cameras_[i].image_queue.front();
    cameras_[i].image_queue.pop();
  }
  
  if (success) {
    sync_frame_counter_.store(target_frame);
    
    // Log sync status occasionally
    static int sync_count = 0;
    if (++sync_count % 100 == 0) {
      std::cout << "[SYNC] Successfully synchronized frame " << target_frame << std::endl;
    }
  }
  
  return success;
}

bool CameraManager::waitForFrameSync(uint64_t target_frame) {
  const int max_wait_ms = 100;  // Maximum wait time
  const int check_interval_us = 1000;  // Check every 1ms
  int total_wait_us = 0;
  
  while (total_wait_us < max_wait_ms * 1000) {
    bool all_ready = true;
    
    for (const auto& cam : cameras_) {
      if (cam.frame_counter.load() < target_frame) {
        all_ready = false;
        break;
      }
    }
    
    if (all_ready) {
      return true;
    }
    
    std::this_thread::sleep_for(std::chrono::microseconds(check_interval_us));
    total_wait_us += check_interval_us;
  }
  
  // Log sync failure
  std::cout << "[SYNC] Frame sync timeout for frame " << target_frame << std::endl;
  for (size_t i = 0; i < cameras_.size(); ++i) {
    std::cout << "  Camera " << i << " frame: " << cameras_[i].frame_counter.load() << std::endl;
  }
  
  return false;
}

double CameraManager::getAverageFrameRate() const {
  if (cameras_.empty()) {
    return 30.0;  // Default fallback
  }
  
  double total_fps = 0.0;
  for (const auto& cam : cameras_) {
    total_fps += cam.params.frame_rate;
  }
  
  return total_fps / cameras_.size();
}

void *CameraManager::getHandle(size_t index) const {
  if (index >= cameras_.size())
    return nullptr;
  return cameras_[index].handle;
}

int CameraManager::setParameter(void *dev_handle_, CameraParams &config) {
  int ret;

  // 设置像素格式为8位BayerRG
  ret = MV_CC_SetEnumValue(dev_handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
  if (ret != MV_OK) {
    std::cerr << "[WARN] Set PixelFormat to BayerRG8 failed: 0x" << std::hex << ret << std::dec << std::endl;
  } else {
    std::cout << "[INFO] Set PixelFormat to BayerRG8 successfully" << std::endl;
  }

  if (config.exposure_auto) {
    _MVCC_FLOATVALUE_T val;
    ret = MV_CC_SetIntValueEx(dev_handle_, "AutoExposureTimeLowerLimit",
                              config.auto_exposure_min);
    if (ret != MV_OK)
      std::cerr << "[WARN] AutoExposureTimeLowerLimit failed: 0x" << std::hex
                << ret << std::dec << std::endl;

    ret = MV_CC_SetIntValueEx(dev_handle_, "AutoExposureTimeUpperLimit",
                              config.auto_exposure_max);
    if (ret != MV_OK)
      std::cerr << "[WARN] AutoExposureTimeUpperLimit failed: 0x" << std::hex
                << ret << std::dec << std::endl;

    ret = MV_CC_SetEnumValue(dev_handle_, "ExposureAuto",
                             MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
    if (ret != MV_OK)
      std::cerr << "[WARN] ExposureAuto CONTINUOUS failed: 0x" << std::hex
                << ret << std::dec << std::endl;

    ret = MV_CC_GetFloatValue(dev_handle_, "ExposureTime", &val);
    if (ret == MV_OK) {
      config.exposure_value = val.fCurValue;
    } else {
      std::cerr << "[WARN] Get ExposureTime failed: 0x" << std::hex << ret
                << std::dec << std::endl;
    }
  } else {
    ret = MV_CC_SetEnumValue(dev_handle_, "ExposureAuto",
                             MV_EXPOSURE_AUTO_MODE_OFF);
    if (ret != MV_OK)
      std::cerr << "[WARN] ExposureAuto OFF failed: 0x" << std::hex << ret
                << std::dec << std::endl;

    ret =
        MV_CC_SetFloatValue(dev_handle_, "ExposureTime", config.exposure_value);
    if (ret != MV_OK)
      std::cerr << "[WARN] Set ExposureTime failed: 0x" << std::hex << ret
                << std::dec << std::endl;
  }

  if (config.gain_auto) {
    _MVCC_FLOATVALUE_T val{};
    ret = MV_CC_SetFloatValue(dev_handle_, "AutoGainLowerLimit",
                              config.auto_gain_min);
    if (ret != MV_OK)
      std::cerr << "[WARN] AutoGainLowerLimit failed: 0x" << std::hex << ret
                << std::dec << std::endl;

    ret = MV_CC_SetFloatValue(dev_handle_, "AutoGainUpperLimit",
                              config.auto_gain_max);
    if (ret != MV_OK)
      std::cerr << "[WARN] AutoGainUpperLimit failed: 0x" << std::hex << ret
                << std::dec << std::endl;

    ret = MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_CONTINUOUS);
    if (ret != MV_OK)
      std::cerr << "[WARN] GainAuto CONTINUOUS failed: 0x" << std::hex << ret
                << std::dec << std::endl;

    ret = MV_CC_GetFloatValue(dev_handle_, "Gain", &val);
    if (ret == MV_OK) {
      config.gain_value = val.fCurValue;
    } else {
      std::cerr << "[WARN] Get Gain failed: 0x" << std::hex << ret << std::dec
                << std::endl;
    }
  } else {
    _MVCC_FLOATVALUE_T val;
    ret = MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_OFF);
    if (ret != MV_OK)
      std::cerr << "[WARN] GainAuto OFF failed: 0x" << std::hex << ret
                << std::dec << std::endl;

    ret = MV_CC_SetFloatValue(dev_handle_, "Gain", config.gain_value);
    if (ret != MV_OK)
      std::cerr << "[WARN] Set Gain failed: 0x" << std::hex << ret << std::dec
                << std::endl;

    ret = MV_CC_GetFloatValue(dev_handle_, "Gain", &val);
    if (ret == MV_OK) {
      config.gain_value = val.fCurValue;
    } else {
      std::cerr << "[WARN] Get Gain failed: 0x" << std::hex << ret << std::dec
                << std::endl;
    }
  }

  if (config.balance_white_auto) {
    ret = MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto",
                             MV_BALANCEWHITE_AUTO_CONTINUOUS);
    if (ret != MV_OK)
      std::cerr << "[WARN] WhiteAuto CONTINUOUS failed: 0x" << std::hex << ret
                << std::dec << std::endl;
  }
  else{
    ret = MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto",
                             MV_BALANCEWHITE_AUTO_OFF);
    if (ret != MV_OK)
      std::cerr << "[WARN] Disable WhiteAuto failed: 0x" << std::hex << ret
                << std::dec << std::endl;
  }

  switch (config.gamma_selector) {
    case 1:
      ret = MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true);
      if (ret != MV_OK)
        std::cerr << "[WARN] Set GammaEnable failed: 0x" << std::hex << ret
                  << std::dec << std::endl;
      ret = MV_CC_SetEnumValue(dev_handle_, "GammaSelector",
                               MV_GAMMA_SELECTOR_USER);
      if (ret != MV_OK)
        std::cerr << "[WARN] GammaSelector USER failed: 0x" << std::hex << ret
                  << std::dec << std::endl;
      ret = MV_CC_SetGamma(dev_handle_, config.gamma_value);
      if (ret != MV_OK)
        std::cerr << "[WARN] SetGamma failed: 0x" << std::hex << ret << std::dec
                  << std::endl;
      break;
    case 2:
      ret = MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true);
      if (ret != MV_OK)
        std::cerr << "[WARN] Set GammaEnable failed: 0x" << std::hex << ret
                  << std::dec << std::endl;
      ret = MV_CC_SetEnumValue(dev_handle_, "GammaSelector",
                               MV_GAMMA_SELECTOR_SRGB);
      if (ret != MV_OK)
        std::cerr << "[WARN] GammaSelector sRGB failed: 0x" << std::hex << ret
                  << std::dec << std::endl;
      break;
    default:
      ret = MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false);
      if (ret != MV_OK)
        std::cerr << "[WARN] Set GammaEnable failed: 0x" << std::hex << ret
                  << std::dec << std::endl;
      break;
  }

  ret = MV_CC_SetIntValueEx(dev_handle_, "Width", config.width);
  if (ret != MV_OK)
    std::cerr << "[WARN] Set Width failed: 0x" << std::hex << ret << std::dec
              << std::endl;

  ret = MV_CC_SetIntValueEx(dev_handle_, "Height", config.height);
  if (ret != MV_OK)
    std::cerr << "[WARN] Set Height failed: 0x" << std::hex << ret << std::dec
              << std::endl;

  ret = MV_CC_SetIntValueEx(dev_handle_, "OffsetX", config.offset_x);
  if (ret != MV_OK)
    std::cerr << "[WARN] Set OffsetX failed: 0x" << std::hex << ret << std::dec
              << std::endl;

  ret = MV_CC_SetIntValueEx(dev_handle_, "OffsetY", config.offset_y);
  if (ret != MV_OK)
    std::cerr << "[WARN] Set OffsetY failed: 0x" << std::hex << ret << std::dec
              << std::endl;

  return MV_OK;
}