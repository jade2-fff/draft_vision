/**
 * @file hikrobot.cpp
 * @brief 海康工业相机封装实现
 */
#include "hikrobot.h"

#include <iostream>
#include <unordered_map>

#include <libusb-1.0/libusb.h>

#include "MvCameraControl.h"

using namespace std::chrono_literals;

HikRobot::HikRobot(double exposure_ms, double gain, const std::string &vid_pid)
    : exposure_us_(exposure_ms * 1e3), gain_(gain) {
    set_vid_pid(vid_pid);
    if (libusb_init(nullptr)) std::cerr << "[HikRobot] libusb init failed" << std::endl;

    daemon_thread_ = std::thread([this] {
        std::cout << "[HikRobot] daemon thread started" << std::endl;
        capture_start();
        while (!daemon_quit_) {
            std::this_thread::sleep_for(100ms);
            if (capturing_) continue;
            capture_stop();
            reset_usb();
            capture_start();
        }
        capture_stop();
        std::cout << "[HikRobot] daemon thread stopped" << std::endl;
    });
}

HikRobot::~HikRobot() {
    daemon_quit_ = true;
    if (daemon_thread_.joinable()) daemon_thread_.join();
}

void HikRobot::read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp) {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    queue_cv_.wait(lk, [this]{ return !queue_.empty(); });
    CameraData data = std::move(queue_.front());
    queue_.pop_front();
    img = data.img;
    timestamp = data.timestamp;
}

bool HikRobot::read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp, int timeout_ms) {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    if (!queue_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                            [this]{ return !queue_.empty(); }))
        return false;
    CameraData data = std::move(queue_.front());
    queue_.pop_front();
    img = data.img;
    timestamp = data.timestamp;
    return true;
}

void HikRobot::capture_start() {
    capturing_   = false;
    capture_quit_ = false;

    MV_CC_DEVICE_INFO_LIST device_list{};
    unsigned int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) { std::cerr << "[HikRobot] EnumDevices failed: 0x" << std::hex << ret << std::endl; return; }
    if (device_list.nDeviceNum == 0) { std::cerr << "[HikRobot] no camera found" << std::endl; return; }

    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) { std::cerr << "[HikRobot] CreateHandle failed: 0x" << std::hex << ret << std::endl; return; }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) { std::cerr << "[HikRobot] OpenDevice failed: 0x" << std::hex << ret << std::endl; return; }

    set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
    set_enum_value("TriggerMode", MV_TRIGGER_MODE_OFF);
    set_float_value("ExposureTime", exposure_us_);
    set_float_value("Gain", gain_);
    MV_CC_SetFrameRate(handle_, 150);

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) { std::cerr << "[HikRobot] StartGrabbing failed: 0x" << std::hex << ret << std::endl; return; }

    std::cout << "[HikRobot] opened, exposure=" << exposure_us_ << "us gain=" << gain_ << std::endl;

    capture_thread_ = std::thread([this] {
        capturing_ = true;
        MV_FRAME_OUT raw{};
        while (!capture_quit_) {
            std::this_thread::sleep_for(1ms);
            unsigned int ret = MV_CC_GetImageBuffer(handle_, &raw, 100);
            if (ret != MV_OK) { std::cerr << "[HikRobot] GetImageBuffer failed: 0x" << std::hex << ret << std::endl; break; }

            auto timestamp = std::chrono::steady_clock::now();
            cv::Mat src(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

            // Bayer → BGR
            static const std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
                {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
                {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
                {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
                {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
            cv::Mat dst;
            cv::cvtColor(src, dst, type_map.at(raw.stFrameInfo.enPixelType));

            {
                std::lock_guard<std::mutex> lk(queue_mtx_);
                if (queue_.size() >= 1) queue_.pop_front();
                queue_.push_back({dst, timestamp});
            }
            queue_cv_.notify_one();

            MV_CC_FreeImageBuffer(handle_, &raw);
        }
        capturing_ = false;
        std::cout << "[HikRobot] capture thread stopped" << std::endl;
    });
}

void HikRobot::capture_stop() {
    capture_quit_ = true;
    if (capture_thread_.joinable()) capture_thread_.join();
    if (!handle_) return;

    unsigned int ret;
    ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK) std::cerr << "[HikRobot] StopGrabbing failed: 0x" << std::hex << ret << std::endl;
    ret = MV_CC_CloseDevice(handle_);
    if (ret != MV_OK) std::cerr << "[HikRobot] CloseDevice failed: 0x" << std::hex << ret << std::endl;
    ret = MV_CC_DestroyHandle(handle_);
    if (ret != MV_OK) std::cerr << "[HikRobot] DestroyHandle failed: 0x" << std::hex << ret << std::endl;
    handle_ = nullptr;
}

void HikRobot::set_float_value(const std::string &name, double value) {
    unsigned int ret = MV_CC_SetFloatValue(handle_, name.c_str(), static_cast<float>(value));
    if (ret != MV_OK) std::cerr << "[HikRobot] SetFloat " << name << "=" << value << " failed: 0x" << std::hex << ret << std::endl;
}

void HikRobot::set_enum_value(const std::string &name, unsigned int value) {
    unsigned int ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);
    if (ret != MV_OK) std::cerr << "[HikRobot] SetEnum " << name << " failed: 0x" << std::hex << ret << std::endl;
}

void HikRobot::set_vid_pid(const std::string &vid_pid) {
    auto index = vid_pid.find(':');
    if (index == std::string::npos) return;
    try {
        vid_ = std::stoi(vid_pid.substr(0, index),   nullptr, 16);
        pid_ = std::stoi(vid_pid.substr(index + 1),  nullptr, 16);
    } catch (...) { vid_ = pid_ = -1; }
}

void HikRobot::reset_usb() const {
    if (vid_ == -1 || pid_ == -1) return;
    auto h = libusb_open_device_with_vid_pid(nullptr, vid_, pid_);
    if (!h) { std::cerr << "[HikRobot] reset_usb: device not found" << std::endl; return; }
    if (libusb_reset_device(h)) std::cerr << "[HikRobot] usb reset failed" << std::endl;
    else                        std::cout << "[HikRobot] usb reset ok" << std::endl;
    libusb_close(h);
}
