/**
 * @file dart_standalone.cpp
 * @brief 飞镖自瞄 — 绿色光源检测 + 串口发送
 *
 * 管线: USB相机 → HSV绿色检测 → 像素偏移算角度 → 双串口 CRC16 发送
 */
#include <chrono>
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>

#include "serial_protocol.h"
#include "dart_detector.h"
#include "dart_aimer.h"

static const char *PORT_GIMBAL  = "/dev/ttyUSB0";
static const char *PORT_CHASSIS = "/dev/ttyACM0";
static const int   BAUD         = B115200;
static const int   CAM_ID       = 0;

int main() {
    cv::VideoCapture cap(CAM_ID);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << std::endl;
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    DartDetector detector;
    DartAimer    aimer;

    int fd_g = serial_open(PORT_GIMBAL, BAUD);
    int fd_c = serial_open(PORT_CHASSIS, BAUD);

    std::cout << (fd_g>=0 ? "[OK] 云台串口" : "[WARN] 云台串口未连接") << std::endl;
    std::cout << (fd_c>=0 ? "[OK] 底盘串口" : "[WARN] 底盘串口未连接") << std::endl;
    std::cout << "[Dart] 绿色光源自瞄启动" << std::endl;

    cv::Mat frame, dbg;
    int fc = 0;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        fc++;

        // 1. 检测
        auto tgt = detector.detect(frame);
        bool found = tgt.confidence > 0;

        // 2. 瞄准
        auto cmd = aimer.aim(
            tgt.cx_norm(frame.cols),
            tgt.cy_norm(frame.rows), found);

        // 3. 发云台
        serial_send_packet(fd_g, cmd.yaw, cmd.pitch, cmd.fire ? 1.0f : 0.0f);
        // 4. 发底盘
        serial_send_packet(fd_c, cmd.yaw, cmd.pitch, 0.0f);

        // 5. 收姿态 (非阻塞)
        float p, r, y;
        serial_recv_packet(fd_g, p, r, y);
        serial_recv_packet(fd_c, p, r, y);

        // 6. 显示
        frame.copyTo(dbg);
        if (found) {
            cv::circle(dbg, tgt.center, int(tgt.radius), {0, 255, 0}, 2);
            cv::circle(dbg, tgt.center, 3, {0, 0, 255}, -1);
        }
        cv::line(dbg, {dbg.cols/2, 0}, {dbg.cols/2, dbg.rows}, {255, 255, 255}, 1);
        cv::line(dbg, {0, dbg.rows/2}, {dbg.cols, dbg.rows/2}, {255, 255, 255}, 1);

        if (fc % 30 == 0) {
            std::cout << "[Dart] #" << fc
                      << (found ? " yaw=" + std::to_string(cmd.yaw).substr(0,6)
                                  + " pitch=" + std::to_string(cmd.pitch).substr(0,6)
                                  + (cmd.fire ? " FIRE" : "")
                                : " NO_TARGET")
                      << std::endl;
        }

        cv::resize(dbg, dbg, {}, 0.5, 0.5);
        cv::imshow("[Dart]", dbg);
        if (cv::waitKey(1) == 'q') break;
    }

    serial_close(fd_g);
    serial_close(fd_c);
    std::cout << "[Dart] 退出。" << std::endl;
    return 0;
}
