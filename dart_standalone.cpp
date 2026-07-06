/**
 * @file dart_standalone.cpp
 * @brief 飞镖自瞄 — 绿色光源检测 + 角度/平面距离解算 + 串口 + 内录 + 可视化
 *
 * 管线: USB相机 → 检测 → PnP平面求交 → 串口发 VisionToGimbal (dx/dy/mode)
 * 单串口: /dev/ttyACM0
 *   上位机→下位机: VisionToGimbal (0x50头, yaw字段=dx_cm, pitch字段=dy_cm, mode)
 *     dx/dy = 目标平面上相对镗准线的真实物理距离(厘米)，右/上为正
 *   下位机→上位机: GimbalToVision (0x53头, yaw/pitch rad, q等)
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "serial_protocol.h"
#include "dart_detector.h"
#include "dart_confirmer.h"
#include "dart_solver.h"
#include "dart_aimer.h"
#include "recorder.h"
#include "io/hikrobot/hikrobot.h"

static const int   BAUD  = B115200;

// 自动检测串口：依次尝试 /dev/ttyACM0..3，返回打开成功的 fd 和端口名
static int serial_open_auto(std::string &opened_port) {
    for (int i = 0; i < 4; ++i) {
        std::string port = "/dev/ttyACM" + std::to_string(i);
        int fd = serial_open(port.c_str(), BAUD);
        if (fd >= 0) { opened_port = port; return fd; }
    }
    opened_port = "/dev/ttyACM0..3";
    return -1;
}

// 海康相机参数
static const double CAM_EXPOSURE_MS = 2.5;
static const double CAM_GAIN        = 7.0;    // MV-CA003 增益上限约 15，实测用 7.0
static const char  *CAM_VID_PID     = "2bdf:0001";

// 串口包录制全局指针（供无捕获函数指针回调访问）
static RawSerialRecorder *g_raw_serial = nullptr;

static const char *state_str(int s) {
    switch (s) {
        case DartConfirmer::LOCKED:    return "LOCKED";
        case DartConfirmer::CANDIDATE: return "CANDIDATE";
        default:                        return "SEARCHING";
    }
}

int main() {
    HikRobot camera(CAM_EXPOSURE_MS, CAM_GAIN, CAM_VID_PID);
    std::cout << "[Dart] 海康相机初始化（守护线程重连中）" << std::endl;

    DartDetector detector;
    DartSolver   solver;
    solver.load_camera("config/dart_intrinsics.yml");     // 真实内参（没有则用默认占位值）
    solver.load_plane_pose("config/dart_plane_pose.yml"); // 外参（依赖内参，先加载内参）
    DartAimer    aimer;

    std::string opened_port;
    int fd = serial_open_auto(opened_port);
    std::cout << (fd>=0 ? "[OK] 串口 " : "[WARN] 串口未连接 ") << opened_port << std::endl;
    std::cout << "[Dart] 飞镖自瞄启动 (r=录像 q=退出)" << std::endl;

    // ── 内录 ──
    auto raw_serial = std::make_unique<RawSerialRecorder>();
    g_raw_serial = raw_serial.get();
    SerialRecorderHooks hooks;
    hooks.on_tx = [](const uint8_t *d, size_t n){ if (g_raw_serial) g_raw_serial->record_tx(d, n); };
    hooks.on_rx = [](const uint8_t *d, size_t n){ if (g_raw_serial) g_raw_serial->record_rx(d, n); };
    serial_set_recorder_hooks(hooks);

    std::unique_ptr<Recorder> recorder = std::make_unique<Recorder>();
    bool recording = true;

    cv::Mat frame, dbg;
    int fc = 0;
    int capture_id = 0;   // 标定截图计数（按 c 保存无 HUD 原图）
    float cur_pitch = 0, cur_yaw = 0, cur_roll = 0;

    // 看门狗：若启动后一直取不到帧（相机卡在坏的资源上下文 0x80000006），
    // 直接退出，靠 systemd Restart=always 拉起一个全新进程重连。
    const auto start_time = std::chrono::steady_clock::now();
    bool got_first_frame = false;

    while (true) {
        std::chrono::steady_clock::time_point t;
        if (!camera.read(frame, t, 500)) {
            // 取帧超时（相机未连/掉线），仍处理按键避免卡死
            if (cv::waitKey(1) == 'q') break;
            // 启动 20 秒内还没拿到第一帧 → 退出让 systemd 重启新进程
            if (!got_first_frame) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start_time).count();
                if (elapsed > 20) {
                    std::cerr << "[Dart] no frame in 20s, exiting for systemd restart"
                              << std::endl;
                    serial_close(fd);
                    return 2;
                }
            }
            continue;
        }
        got_first_frame = true;
        if (frame.empty()) continue;
        fc++;

        // 1. 检测（含 confirmer 确认）
        DartTarget tgt = detector.detect(frame);

        // 2. 解算真角度
        solver.solve(tgt);

        // 3. 瞄准
        AimCommand cmd = aimer.aim(tgt);

        // 4. 收下位机姿态 (pitch/yaw/roll，度；14字节协议)
        {
            float p, y, r;
            if (serial_recv_attitude(fd, p, y, r)) {
                cur_pitch = p; cur_yaw = y; cur_roll = r;
            }
        }

        // 5. 发相机坐标系 x_横向 / y_深度(mm) + valid（14字节协议，×100，0x1021 CRC）
        //    x = 目标相对相机的水平偏移(右为正)，y = 前向深度，原点=相机/发射点。
        //    下位机用 yaw=atan2(x, y) 解算偏航并自行判断到位。垂直分量已丢弃。
        //    z(valid): 1=有目标, 0=无目标。无锁存/死区，直接发实时真实值。
        {
            float x_mm = 0, depth_mm = 0, valid = 0;
            if (tgt.found && tgt.plane_valid) {
                x_mm     = tgt.cam_x_mm;       // 水平偏移，右为正
                depth_mm = tgt.cam_depth_mm;   // 前向深度
                valid    = 1.0f;
            }
            serial_send_plane_offset(fd, x_mm, depth_mm, valid);
        }

        // 6. 可视化 ───────────────────────────────
        frame.copyTo(dbg);
        cv::Point2f bs = solver.boresight();
        cv::Point   bsi(int(bs.x), int(bs.y));

        if (tgt.found) {
            cv::circle(dbg, tgt.center, int(tgt.radius), {0, 255, 0}, 2);
            cv::circle(dbg, tgt.center, 3, {0, 0, 255}, -1);
            cv::arrowedLine(dbg, bsi, tgt.center, {0, 0, 255}, 1, cv::LINE_AA, 0, 0.1);
        }

        // 中心十字
        cv::line(dbg, {dbg.cols/2, 0}, {dbg.cols/2, dbg.rows}, {255, 255, 255}, 1);
        cv::line(dbg, {0, dbg.rows/2}, {dbg.cols, dbg.rows/2}, {255, 255, 255}, 1);

        // 镗准线（绿色十字 + BO 标签）
        const int cross = 20;
        cv::line(dbg, {bsi.x - cross, bsi.y}, {bsi.x + cross, bsi.y}, {0, 255, 0}, 1);
        cv::line(dbg, {bsi.x, bsi.y - cross}, {bsi.x, bsi.y + cross}, {0, 255, 0}, 1);
        cv::circle(dbg, bsi, 3, {0, 255, 0}, 1);
        cv::putText(dbg, "BO", {bsi.x + 5, bsi.y - 5}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 255, 0}, 1, cv::LINE_AA);

        // HUD
        int st = detector.state();
        auto put = [&](int row, const std::string &s, const cv::Scalar &c = {255,255,255}) {
            cv::putText(dbg, s, {10, 25 + row * 22}, cv::FONT_HERSHEY_SIMPLEX, 0.55, c, 1, cv::LINE_AA);
        };
        cv::Scalar st_color = (st == DartConfirmer::LOCKED) ? cv::Scalar{0,255,0}
                        : (st == DartConfirmer::CANDIDATE) ? cv::Scalar{0,255,255}
                        : cv::Scalar{0,165,255};
        put(0, std::string("State: ") + state_str(st), st_color);
        put(1, "Conf:  " + std::to_string(tgt.confidence).substr(0,4));
        put(2, "Yaw:   " + std::to_string(cmd.yaw_err).substr(0,6) + " deg",
            cmd.fire ? cv::Scalar{0,255,0} : cv::Scalar{180,180,180});
        put(3, "Pitch: " + std::to_string(cmd.pitch_err).substr(0,6) + " deg",
            cmd.fire ? cv::Scalar{0,255,0} : cv::Scalar{180,180,180});
        {
            float yaw_deg = std::atan2(tgt.cam_x_mm, tgt.cam_depth_mm) * 180.0f / float(M_PI);
            put(4, "X/Depth: " + std::to_string(tgt.cam_x_mm).substr(0,7) + " "
                      + std::to_string(tgt.cam_depth_mm).substr(0,8) + " mm");
            put(5, "Yaw(atan2): " + std::to_string(yaw_deg).substr(0,6) + " deg  PV:"
                      + std::string(tgt.plane_valid ? "1" : "0"),
                tgt.plane_valid ? cv::Scalar{0,255,0} : cv::Scalar{120,120,120});
        }
        put(6, "Fire:  " + std::string(cmd.fire ? "ON" : "OFF"),
            cmd.fire ? cv::Scalar{0,0,255} : cv::Scalar{120,120,120});
        put(7, "IMU P/Y: " + std::to_string(cur_pitch).substr(0,5) + " "
                  + std::to_string(cur_yaw).substr(0,5) + " deg");
        put(8, std::string("REC: ") + (recording ? "ON " : "OFF") + " [r toggles]",
            recording ? cv::Scalar{0,0,255} : cv::Scalar{120,120,120});
        put(9, "Exp/Gain: " + std::to_string(camera.exposure_ms()).substr(0,5) + "ms "
                  + std::to_string(camera.gain()).substr(0,4)
                  + "  [+/- exp, ][ gain]");
        put(10, "[c] save calib frame -> captures/");

        // 录像（带 HUD 的画面）
        if (recording && recorder)
            recorder->record(dbg, cmd.yaw_err, cmd.pitch_err, cmd.fire ? 1.0f : 0.0f, tgt.confidence);

        if (fc % 30 == 0) {
            // 串口发送 x_横向/y_深度(mm)；此处同时打印 yaw=atan2(x,y) 便于验证
            float yaw_deg = std::atan2(tgt.cam_x_mm, tgt.cam_depth_mm) * 180.0f / float(M_PI);
            std::cout << "[Dart] #" << fc
                      << (tgt.found ? " x_mm=" + std::to_string(tgt.cam_x_mm).substr(0,7)
                                  + " depth_mm=" + std::to_string(tgt.cam_depth_mm).substr(0,8)
                                  + " yaw=" + std::to_string(yaw_deg).substr(0,6)
                                  + " pv=" + std::string(tgt.plane_valid ? "1" : "0")
                                : " NO_TARGET")
                      << std::endl;
        }

        cv::imshow("[Dart]", dbg);
        const cv::Mat &mask = detector.debug_mask();
        if (!mask.empty()) cv::imshow("[dart] mask", mask);

        int key = cv::waitKey(1);
        if (key == 'q') break;
        if (key == '+' || key == '=') {
            camera.set_exposure(camera.exposure_ms() + 2.0);
            std::cout << "[Cam] exposure=" << camera.exposure_ms()
                      << "ms gain=" << camera.gain() << std::endl;
        }
        if (key == '-' || key == '_') {
            camera.set_exposure(camera.exposure_ms() - 2.0);
            std::cout << "[Cam] exposure=" << camera.exposure_ms()
                      << "ms gain=" << camera.gain() << std::endl;
        }
        if (key == ']') {
            camera.set_gain(camera.gain() + 2.0);
            std::cout << "[Cam] exposure=" << camera.exposure_ms()
                      << "ms gain=" << camera.gain() << std::endl;
        }
        if (key == '[') {
            camera.set_gain(camera.gain() - 2.0);
            std::cout << "[Cam] exposure=" << camera.exposure_ms()
                      << "ms gain=" << camera.gain() << std::endl;
        }
        if (key == 'c') {
            // 保存无 HUD 的原始帧，供 calibrate_plane_pose 使用
            system("mkdir -p captures");
            char path[128];
            std::snprintf(path, sizeof(path), "captures/capture_%02d.jpg", capture_id++);
            if (cv::imwrite(path, frame))
                std::cout << "[main] Saved calibration frame: " << path << std::endl;
            else
                std::cerr << "[main] Failed to save: " << path << std::endl;
        }
        if (key == 'r') {
            recording = !recording;
            if (recording) {
                recorder = std::make_unique<Recorder>();
                std::cout << "[main] Recording STARTED" << std::endl;
            } else {
                recorder.reset();
                std::cout << "[main] Recording STOPPED" << std::endl;
            }
        }
    }

    serial_close(fd);
    std::cout << "[Dart] 退出。" << std::endl;
    return 0;
}
