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
  // Use move semantics to avoid unnecessary copy
  image = std::move(cam.image_queue.front());
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
  static auto last_time = std::chrono::steady_clock::now();
  
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
  
  if (elapsed >= 5000) {  // Report every 5 seconds
    // int current_count = g_callback_count.load();
    // static int last_count = 0;
    // double fps = (current_count - last_count) * 1000.0 / elapsed;
    // std::cout << "[CALLBACK] FPS: " << fps << " (total: " << current_count << ")" << std::endl;
    // last_count = current_count;
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

  if (info->enPixelType == PixelType_Gvsp_BayerRG8) {
    // Optimize Bayer conversion: use in-place conversion if possible
    // Allocate output directly to avoid temporary Mat
    img.create(H, W, CV_8UC3);
    cv::Mat bayer_img(H, W, CV_8UC1, const_cast<unsigned char*>(data));
    // Use fast interpolation (0 = nearest, which is fastest)
    cv::cvtColor(bayer_img, img, cv::COLOR_BayerBG2BGR, 0); 
  }
  else if (info->enPixelType == PixelType_Gvsp_Mono8) {
    // Clone is necessary here as data pointer is temporary
    img = cv::Mat(H, W, CV_8UC1, data).clone();
  }
  else {
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
      // Avoid clone - create Mat that references the buffer
      // We'll move it to queue so ownership is transferred
      img = cv::Mat(H, W, CV_8UC3, ctx.cvt_buf.data()).clone();  // Still need clone as buffer may be reused
    } else {
      std::cerr << "[WARN] SDK conversion failed: 0x" << std::hex << ret << std::dec << std::endl;
      img = cv::Mat(H, W, CV_8UC1, const_cast<unsigned char*>(data)).clone();
    }
  }

  if (img.empty()) {
    std::cerr << "[ERROR] Failed to create image!" << std::endl;
    return;
  }

  // Apply vignetting correction if enabled
  if (ctx.params.vignetting_enable) {
    static int vignetting_count = 0;
    vignetting_count++;
    if (vignetting_count % 30 == 0) { // Log every 30 frames
      std::cout << "[VIGNETTING] Applying correction: a=" << ctx.params.vignetting_a 
                << ", b=" << ctx.params.vignetting_b 
                << ", c=" << ctx.params.vignetting_c << std::endl;
      
      // Debug: Check image statistics before and after correction
      cv::Scalar mean_before = cv::mean(img);
      cv::Mat corrected = applyVignettingCorrection(img, ctx.params.vignetting_a, ctx.params.vignetting_b, ctx.params.vignetting_c);
      cv::Scalar mean_after = cv::mean(corrected);
      std::cout << "[VIGNETTING] Image mean before: " << mean_before << ", after: " << mean_after << std::endl;
      
      // Debug: Check if there's any difference
      cv::Mat diff;
      cv::absdiff(img, corrected, diff);
      double max_diff = 0;
      cv::minMaxLoc(diff, nullptr, &max_diff);
      std::cout << "[VIGNETTING] Max difference: " << max_diff << std::endl;
      
      img = corrected;
    } else {
      img = applyVignettingCorrection(img, ctx.params.vignetting_a, ctx.params.vignetting_b, ctx.params.vignetting_c);
    }
  }

  // Performance tracking (commented out to reduce overhead)
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // static int process_count = 0;
  // process_count++;
  // if (process_count % 100 == 0) {
  //   std::cout << "[PROCESS] Image processing took: " << duration.count() << " μs" << std::endl;
  // }
  (void)start_time;  // Suppress unused variable warning

  std::lock_guard<std::mutex> lock(ctx.mtx);
  
  // Low-latency optimization: Keep queue very small (2-3 frames max)
  // This minimizes delay between capture and consumption
  // Old frames are dropped immediately to always use latest
  const size_t MAX_QUEUE_SIZE = 2;  // Reduced from 30 to 2 for minimal latency
  while (ctx.image_queue.size() >= MAX_QUEUE_SIZE) {
    ctx.image_queue.pop();  // Drop oldest frame immediately
  }
  
  // Move image into queue (img will be empty after move, which is fine)
  // This avoids one clone operation
  ctx.image_queue.push(std::move(img));
}

bool CameraManager::getSyncedImages(std::vector<cv::Mat> &images) {
  if (cameras_.size() <= 1) {
    // For single camera, just get the image normally
    images.resize(1);
    return getImage(0, images[0]);
  }
  
  // For multiple cameras, get latest available images from each queue
  // No waiting - just get what's available to maximize throughput
  images.resize(cameras_.size());
  bool success = true;
  
  // Lock all cameras at once to get consistent state
  std::vector<std::unique_lock<std::mutex>> locks;
  locks.reserve(cameras_.size());
  for (auto& cam : cameras_) {
    locks.emplace_back(cam.mtx);
  }
  
  // Check if all queues have at least one image
  for (size_t i = 0; i < cameras_.size(); ++i) {
    if (cameras_[i].image_queue.empty()) {
      success = false;
      break;
    }
  }
  
  if (success) {
    // All cameras have images, get them
    for (size_t i = 0; i < cameras_.size(); ++i) {
      images[i] = std::move(cameras_[i].image_queue.front());
      cameras_[i].image_queue.pop();
    }
  }
  
  return success;
}

// Removed waitForFrameSync - no longer needed with non-blocking approach
// This function kept for compatibility but should not be called
bool CameraManager::waitForFrameSync(uint64_t target_frame) {
  (void)target_frame;  // Suppress unused parameter warning
  // Non-blocking approach - always return true to avoid waiting
  return true;
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

cv::Mat CameraManager::generateVignettingMask(const cv::Size& size, float a, float b, float c) {
  int h = size.height;
  int w = size.width;
  
  // Create coordinate matrices like np.indices()
  cv::Mat y_coords, x_coords;
  cv::Mat y_range = cv::Mat::zeros(h, 1, CV_32F);
  cv::Mat x_range = cv::Mat::zeros(1, w, CV_32F);
  
  for (int i = 0; i < h; ++i) {
    y_range.at<float>(i, 0) = static_cast<float>(i);
  }
  for (int j = 0; j < w; ++j) {
    x_range.at<float>(0, j) = static_cast<float>(j);
  }
  
  // Repeat to create full coordinate matrices
  cv::repeat(y_range, 1, w, y_coords);
  cv::repeat(x_range, h, 1, x_coords);
  
  // Calculate center
  float cx = static_cast<float>(w) / 2.0f;
  float cy = static_cast<float>(h) / 2.0f;
  
  // Calculate radius matrix
  cv::Mat dx = x_coords - cx;
  cv::Mat dy = y_coords - cy;
  cv::Mat r;
  cv::magnitude(dx, dy, r);
  
  // Normalize by max radius (like np.max(r))
  double max_r;
  cv::minMaxLoc(r, nullptr, &max_r);
  cv::Mat r_norm = r / max_r;
  
  // Calculate mask using the same formula as Python
  cv::Mat r_norm_2, r_norm_4, r_norm_6;
  cv::pow(r_norm, 2, r_norm_2);
  cv::pow(r_norm, 4, r_norm_4);
  cv::pow(r_norm, 6, r_norm_6);
  
  cv::Mat mask = 1.0f + a * r_norm_2 + b * r_norm_4 + c * r_norm_6;
  
  return mask;
}

cv::Mat CameraManager::applyVignettingCorrection(const cv::Mat& image, float a, float b, float c) {
  if (a == 0.0f && b == 0.0f && c == 0.0f) {
    return image.clone(); // No correction needed
  }
  
  cv::Mat mask = generateVignettingMask(image.size(), a, b, c);
  
  // Convert to float32 like Python version
  cv::Mat img_f32;
  image.convertTo(img_f32, CV_32F);
  
  // Apply correction (division like Python version)
  cv::Mat corrected;
  cv::divide(img_f32, mask, corrected);
  
  // Clip to [0, 255] like Python version
  cv::Mat clipped;
  cv::threshold(corrected, clipped, 255.0, 255.0, cv::THRESH_TRUNC);
  cv::threshold(clipped, clipped, 0.0, 0.0, cv::THRESH_TOZERO);
  
  // Convert back to uint8
  cv::Mat result;
  clipped.convertTo(result, CV_8U);
  
  return result;
}

bool CameraManager::updateCameraParams(size_t camera_index, const CameraParams& new_params) {
  std::cout << "[DEBUG] updateCameraParams called for camera " << camera_index << std::endl;
  std::cout << "[DEBUG] vignetting_enable: " << (new_params.vignetting_enable ? "true" : "false") << std::endl;
  std::cout << "[DEBUG] vignetting_a: " << new_params.vignetting_a << std::endl;
  std::cout << "[DEBUG] vignetting_b: " << new_params.vignetting_b << std::endl;
  std::cout << "[DEBUG] vignetting_c: " << new_params.vignetting_c << std::endl;
  
  if (camera_index >= cameras_.size()) {
    std::cerr << "[ERROR] Camera index " << camera_index << " out of range (0-" << cameras_.size()-1 << ")" << std::endl;
    return false;
  }
  
  auto& ctx = cameras_[camera_index];
  
  // Store old values for comparison
  bool old_enable = ctx.params.vignetting_enable;
  float old_a = ctx.params.vignetting_a;
  float old_b = ctx.params.vignetting_b;
  float old_c = ctx.params.vignetting_c;
  
  // Update the parameters in the camera context
  ctx.params = new_params;
  
  // Log vignetting parameter changes
  if (new_params.vignetting_enable != old_enable ||
      new_params.vignetting_a != old_a ||
      new_params.vignetting_b != old_b ||
      new_params.vignetting_c != old_c) {
    std::cout << "[VIGNETTING] Updated camera " << camera_index << " vignetting params:" << std::endl;
    std::cout << "  enable: " << (new_params.vignetting_enable ? "true" : "false") << std::endl;
    std::cout << "  a: " << new_params.vignetting_a << std::endl;
    std::cout << "  b: " << new_params.vignetting_b << std::endl;
    std::cout << "  c: " << new_params.vignetting_c << std::endl;
  }
  
  // Apply hardware parameters if camera handle is available
  if (ctx.handle) {
    int ret = setParameter(ctx.handle, ctx.params);
    if (ret != 0) { // MV_OK = 0
      std::cerr << "[WARN] Failed to update camera " << camera_index << " parameters: 0x" << std::hex << ret << std::dec << std::endl;
      return false;
    }
  }
  
  std::cout << "[INFO] Successfully updated camera " << camera_index << " parameters" << std::endl;
  return true;
}