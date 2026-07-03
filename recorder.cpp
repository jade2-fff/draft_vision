/**
 * @file recorder.cpp
 * @brief 内录实现 — 视频/文本/串口包，异步线程 + 断电恢复
 */
#include "recorder.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

// ── 时间戳文件名 ──
static std::string timestamp_str() {
    std::time_t t = std::time(nullptr);
    std::tm     tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm);
    return buf;
}

static double steady_now_sec() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ═══════════════════════════════════════════
// Recorder
// ═══════════════════════════════════════════
Recorder::Recorder(double fps) : fps_(fps) {
    start_time_ = std::chrono::steady_clock::now();
    last_time_  = start_time_;

    const std::string folder = "records";
    const std::string name   = timestamp_str();
    text_path_   = folder + "/" + name + ".txt";
    video_path_  = folder + "/" + name + ".avi";
    frames_dir_  = folder + "/" + name + "_frames";

    fs::create_directory(folder);
    fs::create_directory(frames_dir_);

    recover_orphaned_frames(folder);   // 恢复上次断电遗留的帧
}

Recorder::~Recorder() {
    stop_thread_ = true;
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        queue_.push_back(FrameData{});   // 空帧唤醒
    }
    queue_cv_.notify_one();
    if (saving_thread_.joinable()) saving_thread_.join();

    if (!init_) return;
    text_writer_.close();
    video_writer_.release();

    // 正常退出：ffmpeg 合并分帧 -> mp4，成功则删帧目录
    std::string mp4_path = video_path_;
    mp4_path.replace(mp4_path.rfind(".avi"), 4, ".mp4");
    char cmd[512];
    std::snprintf(cmd, sizeof(cmd),
        "ffmpeg -y -framerate %d -i %s/%%06d.jpg -c:v libx264 -pix_fmt yuv420p %s > /dev/null 2>&1",
        static_cast<int>(fps_), frames_dir_.c_str(), mp4_path.c_str());
    if (std::system(cmd) == 0) {
        fs::remove_all(frames_dir_);
        std::cout << "[Recorder] Merged -> " << mp4_path << std::endl;
    } else {
        std::cerr << "[Recorder] ffmpeg merge failed, frames kept in " << frames_dir_ << std::endl;
    }
}

void Recorder::recover_orphaned_frames(const std::string &folder) {
    for (const auto &entry : fs::directory_iterator(folder)) {
        if (!entry.is_directory()) continue;
        std::string dir = entry.path().string();
        if (dir.find("_frames") == std::string::npos) continue;

        // 空目录直接删
        if (fs::is_empty(entry.path())) { fs::remove(entry.path()); continue; }

        std::string mp4 = dir;
        mp4.replace(mp4.rfind("_frames"), 7, ".mp4");
        char cmd[512];
        std::snprintf(cmd, sizeof(cmd),
            "ffmpeg -y -framerate 30 -i %s/%%06d.jpg -c:v libx264 -pix_fmt yuv420p %s > /dev/null 2>&1",
            dir.c_str(), mp4.c_str());
        if (std::system(cmd) == 0) {
            fs::remove_all(entry.path());
            std::cout << "[Recorder] Recovered orphaned frames -> " << mp4 << std::endl;
        }
    }
}

void Recorder::init(const cv::Mat &img) {
    text_writer_.open(text_path_);
    auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    video_writer_.open(video_path_, fourcc, fps_, img.size());
    saving_thread_ = std::thread(&Recorder::save_to_file, this);
    init_ = true;
}

void Recorder::record(const cv::Mat &img, float yaw, float pitch, float fire, float conf) {
    if (img.empty()) return;
    if (!init_) init(img);

    auto now = std::chrono::steady_clock::now();
    double since_last = std::chrono::duration<double>(now - last_time_).count();
    if (since_last < 1.0 / fps_) return;
    last_time_ = now;

    FrameData f;
    img.copyTo(f.img);
    f.yaw = yaw; f.pitch = pitch; f.fire = fire; f.conf = conf;
    f.since_begin = std::chrono::duration<double>(now - start_time_).count();

    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        if (queue_.size() >= 1) queue_.pop_front();   // 容量 1，丢最旧
        queue_.push_back(std::move(f));
    }
    queue_cv_.notify_one();
}

void Recorder::save_to_file() {
    while (true) {
        FrameData f;
        {
            std::unique_lock<std::mutex> lk(queue_mtx_);
            queue_cv_.wait(lk, [this]{ return !queue_.empty(); });
            f = std::move(queue_.front());
            queue_.pop_front();
        }
        if (f.img.empty()) {   // 哨兵：退出
            if (stop_thread_) break;
            continue;
        }
        video_writer_.write(f.img);

        char frame_path[256];
        std::snprintf(frame_path, sizeof(frame_path), "%s/%06ld.jpg",
                      frames_dir_.c_str(), frame_idx_++);
        cv::imwrite(frame_path, f.img);

        if (text_writer_.is_open()) {
            text_writer_ << f.since_begin << ' '
                         << f.yaw << ' ' << f.pitch << ' '
                         << f.fire << ' ' << f.conf << '\n';
        }
    }
}

// ═══════════════════════════════════════════
// RawSerialRecorder
// ═══════════════════════════════════════════
RawSerialRecorder::RawSerialRecorder() {
    const std::string folder = "records";
    fs::create_directory(folder);
    const std::string name = timestamp_str();
    rx_path_ = folder + "/" + name + "_rx.bin";
    tx_path_ = folder + "/" + name + "_tx.bin";
    saving_thread_ = std::thread(&RawSerialRecorder::save_to_file, this);
}

RawSerialRecorder::~RawSerialRecorder() {
    stop_thread_ = true;
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        queue_.push_back(Packet{});   // 空包唤醒
    }
    queue_cv_.notify_one();
    if (saving_thread_.joinable()) saving_thread_.join();
    if (rx_writer_.is_open()) rx_writer_.close();
    if (tx_writer_.is_open()) tx_writer_.close();
}

void RawSerialRecorder::enqueue(Direction dir, const uint8_t *data, size_t size) {
    if (!data || size == 0) return;
    Packet p;
    p.dir = dir;
    p.ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    p.bytes.assign(data, data + size);
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        if (queue_.size() >= 4096) queue_.pop_front();   // 防爆
        queue_.push_back(std::move(p));
    }
    queue_cv_.notify_one();
}

void RawSerialRecorder::record_rx(const uint8_t *data, size_t size) { enqueue(Direction::RX, data, size); }
void RawSerialRecorder::record_tx(const uint8_t *data, size_t size) { enqueue(Direction::TX, data, size); }

void RawSerialRecorder::save_to_file() {
    while (true) {
        Packet p;
        {
            std::unique_lock<std::mutex> lk(queue_mtx_);
            queue_cv_.wait(lk, [this]{ return !queue_.empty(); });
            p = std::move(queue_.front());
            queue_.pop_front();
        }
        if (p.bytes.empty()) {   // 哨兵
            if (stop_thread_) break;
            continue;
        }
        std::ofstream *w = nullptr;
        if (p.dir == Direction::RX) {
            if (!rx_init_) { rx_writer_.open(rx_path_, std::ios::binary | std::ios::out); rx_init_ = true; }
            w = &rx_writer_;
        } else {
            if (!tx_init_) { tx_writer_.open(tx_path_, std::ios::binary | std::ios::out); tx_init_ = true; }
            w = &tx_writer_;
        }
        if (!w || !w->is_open()) continue;
        uint8_t len = static_cast<uint8_t>(std::min<size_t>(p.bytes.size(), 255));
        w->write(reinterpret_cast<const char *>(&p.ts_ns), sizeof(p.ts_ns));
        w->write(reinterpret_cast<const char *>(&len), sizeof(len));
        w->write(reinterpret_cast<const char *>(p.bytes.data()), len);
    }
}
