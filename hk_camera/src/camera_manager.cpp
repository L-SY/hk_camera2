#include "hk_camera/camera_manager.h"
#include <cstring>
#include <iostream>

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

  MV_CC_SetEnumValue(ctx.handle, "TriggerMode", 1);
  MV_CC_SetEnumValue(ctx.handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
  MV_CC_SetBoolValue(ctx.handle, "AcquisitionFrameRateEnable", false);

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
  trigger_thread_ = std::thread([this]() {
    while (running_) {
      triggerAll();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
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
  enqueueImage(*ctx, pData, pFrameInfo);
}

void CameraManager::enqueueImage(CameraContext &ctx,
                                 unsigned char *data,
                                 MV_FRAME_OUT_INFO_EX *info) {
  int W = info->nWidth;
  int H = info->nHeight;
  int L = info->nFrameLen;
  cv::Mat img;

  // 打印像素格式调试信息
  std::cout << "[DEBUG] Camera pixel format: 0x" << std::hex << info->enPixelType << std::dec 
            << ", Width: " << W << ", Height: " << H << ", Length: " << L << std::endl;

  // 处理8位Bayer格式
  if (info->enPixelType == PixelType_Gvsp_BayerRG8) {
    cv::Mat bayer_img(H, W, CV_8UC1, data);
    cv::cvtColor(bayer_img, img, cv::COLOR_BayerBG2BGR);  // 8位BayerRG也用BG模式修正颜色
    std::cout << "[DEBUG] OpenCV 8-bit Bayer RG (as BG) to BGR conversion" << std::endl;
  }
  else if (info->enPixelType == PixelType_Gvsp_BayerGR8) {
    cv::Mat bayer_img(H, W, CV_8UC1, data);
    cv::cvtColor(bayer_img, img, cv::COLOR_BayerRG2BGR);  // GR用RG模式
    std::cout << "[DEBUG] OpenCV 8-bit Bayer GR (as RG) to BGR conversion" << std::endl;
  }
  else if (info->enPixelType == PixelType_Gvsp_BayerGB8) {
    cv::Mat bayer_img(H, W, CV_8UC1, data);
    cv::cvtColor(bayer_img, img, cv::COLOR_BayerGR2BGR);  // GB用GR模式
    std::cout << "[DEBUG] OpenCV 8-bit Bayer GB (as GR) to BGR conversion" << std::endl;
  }
  else if (info->enPixelType == PixelType_Gvsp_BayerBG8) {
    cv::Mat bayer_img(H, W, CV_8UC1, data);
    cv::cvtColor(bayer_img, img, cv::COLOR_BayerGB2BGR);  // BG用GB模式
    std::cout << "[DEBUG] OpenCV 8-bit Bayer BG (as GB) to BGR conversion" << std::endl;
  }
  // 处理10位Bayer格式
  else if (info->enPixelType == PixelType_Gvsp_BayerRG10) {
    // 10位数据需要转换为16位然后缩放到8位
    cv::Mat bayer16(H, W, CV_16UC1, data);
    cv::Mat bayer8;
    bayer16.convertTo(bayer8, CV_8UC1, 1.0/4.0);  // 10位->8位 (除以4)
    cv::cvtColor(bayer8, img, cv::COLOR_BayerBG2BGR);  // 尝试BG模式修正颜色
    std::cout << "[DEBUG] OpenCV 10-bit Bayer RG (as BG) to BGR conversion" << std::endl;
  }
  else if (info->enPixelType == PixelType_Gvsp_BayerGR10) {
    cv::Mat bayer16(H, W, CV_16UC1, data);
    cv::Mat bayer8;
    bayer16.convertTo(bayer8, CV_8UC1, 1.0/4.0);
    cv::cvtColor(bayer8, img, cv::COLOR_BayerRG2BGR);
    std::cout << "[DEBUG] OpenCV 10-bit Bayer GR (as RG) to BGR conversion" << std::endl;
  }
  else if (info->enPixelType == PixelType_Gvsp_BayerGB10) {
    cv::Mat bayer16(H, W, CV_16UC1, data);
    cv::Mat bayer8;
    bayer16.convertTo(bayer8, CV_8UC1, 1.0/4.0);
    cv::cvtColor(bayer8, img, cv::COLOR_BayerGR2BGR);
    std::cout << "[DEBUG] OpenCV 10-bit Bayer GB (as GR) to BGR conversion" << std::endl;
  }
  else if (info->enPixelType == PixelType_Gvsp_BayerBG10) {
    cv::Mat bayer16(H, W, CV_16UC1, data);
    cv::Mat bayer8;
    bayer16.convertTo(bayer8, CV_8UC1, 1.0/4.0);
    cv::cvtColor(bayer8, img, cv::COLOR_BayerGB2BGR);
    std::cout << "[DEBUG] OpenCV 10-bit Bayer BG (as GB) to BGR conversion" << std::endl;
  }
  else if (info->enPixelType == PixelType_Gvsp_Mono8) {
    // 单通道灰度图，直接使用
    img = cv::Mat(H, W, CV_8UC1, data).clone();
    std::cout << "[DEBUG] Using Mono8 format directly" << std::endl;
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
      std::cout << "[DEBUG] SDK BGR conversion successful" << std::endl;
    } else {
      std::cerr << "[WARN] SDK conversion failed: 0x" << std::hex << ret << std::dec << std::endl;
      // 最后的降级方案：直接作为灰度图使用
      img = cv::Mat(H, W, CV_8UC1, data).clone();
      std::cout << "[DEBUG] Using raw data as Mono8 fallback" << std::endl;
    }
  }

  if (img.empty()) {
    std::cerr << "[ERROR] Failed to create image!" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(ctx.mtx);
  ctx.image_queue.push(img.clone());
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