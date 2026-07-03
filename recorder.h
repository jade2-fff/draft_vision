/**
 * @file recorder.h
 * @brief 内录：视频(分帧JPEG→mp4) + 指令文本日志 + 串口收发原始包
 *
 * 照搬 master2 tools/recorder.cpp 形态，纯标准库 + OpenCV 实现，零框架依赖。
 * - Recorder:        异步线程，分帧存 JPEG（断电只丢最后一帧），退出 ffmpeg 合并 mp4
 * - RawSerialRecorder: 串口 rx/tx 原始包带时间戳录二进制
 */
#ifndef DART_RECORDER_H
#define DART_RECORDER_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

// ═══════════════════════════════════════════
// 视频 + 指令文本日志
// ═══════════════════════════════════════════
class Recorder {
public:
    explicit Recorder(double fps = 30.0);
    ~Recorder();

    Recorder(const Recorder &)            = delete;
    Recorder &operator=(const Recorder &) = delete;

    /** 记录一帧（带 HUD 的图像 + 当前指令），内部限频到 fps */
    void record(const cv::Mat &img, float yaw, float pitch, float fire, float conf);

private:
    struct FrameData {
        cv::Mat img;
        float   yaw   = 0;
        float   pitch = 0;
        float   fire  = 0;
        float   conf  = 0;
        double  since_begin = 0;   // 相对启动的秒数
    };

    void init(const cv::Mat &img);
    void save_to_file();
    void recover_orphaned_frames(const std::string &folder);

    bool   init_        = false;
    double fps_         = 30.0;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_time_;

    std::string text_path_;
    std::string video_path_;
    std::string frames_dir_;
    long        frame_idx_ = 0;

    cv::VideoWriter          video_writer_;
    std::ofstream            text_writer_;

    // 有界阻塞队列（容量 1，丢最旧）
    std::deque<FrameData>    queue_;
    std::mutex               queue_mtx_;
    std::condition_variable  queue_cv_;
    std::atomic<bool>        stop_thread_{false};
    std::thread              saving_thread_;
};

// ═══════════════════════════════════════════
// 串口收发原始包录制
// ═══════════════════════════════════════════
class RawSerialRecorder {
public:
    enum class Direction : uint8_t { RX = 0, TX = 1 };

    RawSerialRecorder();
    ~RawSerialRecorder();

    RawSerialRecorder(const RawSerialRecorder &)            = delete;
    RawSerialRecorder &operator=(const RawSerialRecorder &) = delete;

    void record_rx(const uint8_t *data, size_t size);
    void record_tx(const uint8_t *data, size_t size);

private:
    struct Packet {
        Direction    dir;
        int64_t      ts_ns = 0;
        std::vector<uint8_t> bytes;
    };

    void enqueue(Direction dir, const uint8_t *data, size_t size);
    void save_to_file();

    bool                     rx_init_ = false, tx_init_ = false;
    std::atomic<bool>        stop_thread_{false};
    std::deque<Packet>       queue_;
    std::mutex               queue_mtx_;
    std::condition_variable  queue_cv_;
    std::thread              saving_thread_;

    std::string              rx_path_, tx_path_;
    std::ofstream            rx_writer_, tx_writer_;
};

#endif // DART_RECORDER_H
