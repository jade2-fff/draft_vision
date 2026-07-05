/**
 * @file hikrobot.h
 * @brief 海康工业相机封装 — 移植自 master2 io/hikrobot，零框架依赖
 *
 * 用海康 MVS SDK (MvCameraControl) 取流，Bayer→BGR，异步抓帧线程 + 有界队列。
 * 参数硬编码在顶部（曝光/增益/vid:pid），独立部署无需 yaml。
 */
#ifndef DART_HIKROBOT_H
#define DART_HIKROBOT_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

class HikRobot {
public:
    /**
     * @param exposure_ms 曝光时间（毫秒）
     * @param gain        增益
     * @param vid_pid     USB vid:pid（用于掉线重置，如 "2bdf:0001"）
     */
    HikRobot(double exposure_ms = 4.5, double gain = 16.0,
             const std::string &vid_pid = "2bdf:0001");
    ~HikRobot();

    HikRobot(const HikRobot &)            = delete;
    HikRobot &operator=(const HikRobot &) = delete;

    /** 阻塞取一帧 + 时间戳 */
    void read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp);

    /** 带超时取帧，超时返回 false（用于主循环不卡死） */
    bool read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp, int timeout_ms);

    bool is_capturing() const { return capturing_; }

    /** 运行时设曝光（毫秒），钳制到 [0.1,100]，重连后保持 */
    void set_exposure(double exposure_ms);
    /** 运行时设增益，钳制到 [0,40]，重连后保持 */
    void set_gain(double gain);
    double exposure_ms() const { return exposure_us_ / 1e3; }
    double gain() const { return gain_; }

private:
    struct CameraData {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
    };

    void capture_start();
    void capture_stop();
    void set_float_value(const std::string &name, double value);
    void set_enum_value(const std::string &name, unsigned int value);
    void set_vid_pid(const std::string &vid_pid);
    void reset_usb() const;

    std::atomic<double> exposure_us_;
    std::atomic<double> gain_;
    std::mutex          param_mtx_;   // 保护运行时改 handle 参数
    int    vid_ = -1, pid_ = -1;

    void  *handle_ = nullptr;   // MV_CC_HANDLE

    std::atomic<bool> daemon_quit_{false};
    std::atomic<bool> capture_quit_{false};
    std::atomic<bool> capturing_{false};
    std::thread daemon_thread_;
    std::thread capture_thread_;

    // 有界阻塞队列（容量 1，丢最旧）
    std::deque<CameraData>    queue_;
    std::mutex                queue_mtx_;
    std::condition_variable   queue_cv_;
};

#endif
