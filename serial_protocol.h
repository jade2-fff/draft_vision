/**
 * @file serial_protocol.h
 * @brief 串口协议层 — 与下位机 Gimbal 协议完全一致
 *
 * 上位机→下位机: VisionToGimbal (0x50头, 28字节)
 *   字段: head=0x50, mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc, crc16
 *   mode: 0=空闲, 1=控制不开火, 2=控制且开火
 *   yaw/pitch 单位: rad
 *
 * 下位机→上位机: GimbalToVision (0x53头, 43字节)
 *   字段: head=0x53, mode, enemy_color, q[4], yaw, yaw_vel, pitch, pitch_vel,
 *         bullet_speed, bullet_count, crc16
 *
 * CRC16: 多项式 0x1189, 初始值 0xFFFF, 右移查表法
 * 帧尾 CRC 字节序: 小端 (low byte, high byte)
 */
#ifndef DART_SERIAL_PROTOCOL_H
#define DART_SERIAL_PROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <termios.h>

// ── VisionToGimbal — 上位机→下位机 (28 bytes) ──
struct __attribute__((packed)) VisionToGimbal {
    uint8_t  head     = 0x50;
    uint8_t  mode;            // 0:空闲, 1:控制不开火, 2:控制且开火
    float    yaw;             // [rad]
    float    yaw_vel;         // [rad/s]
    float    yaw_acc;         // [rad/s²]
    float    pitch;           // [rad]
    float    pitch_vel;       // [rad/s]
    float    pitch_acc;       // [rad/s²]
    uint16_t crc16;
};

// ── GimbalToVision — 下位机→上位机 (43 bytes) ──
struct __attribute__((packed)) GimbalToVision {
    uint8_t  head = 0x53;
    uint8_t  mode;            // 0:空闲, 1:自瞄, 2:小符, 3:大符
    uint8_t  enemy_color;     // 0:红, 1:蓝, 2:未知
    float    q[4];            // w,x,y,z
    float    yaw;             // [rad]
    float    yaw_vel;         // [rad/s]
    float    pitch;           // [rad]
    float    pitch_vel;       // [rad/s]
    float    bullet_speed;    // [m/s]
    uint16_t bullet_count;
    uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) == 28, "VisionToGimbal size mismatch");
static_assert(sizeof(GimbalToVision) == 43, "GimbalToVision size mismatch");

// ── 串口 ──
int  serial_open(const char *port, int baud);
void serial_close(int fd);

// ── CRC16（多项式 0x1189，与下位机一致） ──
uint16_t crc16_calculate(const uint8_t *data, uint32_t len);

// ── 发送 ──
void serial_send_gimbal_cmd(int fd, uint8_t mode, float yaw, float pitch);

// ── 接收（非阻塞，扫描 0x53 帧头 + CRC 校验） ──
bool serial_recv_gimbal_state(int fd, float &yaw, float &pitch);

// ── 串口包录制钩子 ──
struct SerialRecorderHooks {
    void (*on_tx)(const uint8_t *data, size_t size) = nullptr;
    void (*on_rx)(const uint8_t *data, size_t size) = nullptr;
};
void serial_set_recorder_hooks(const SerialRecorderHooks &hooks);

#endif // DART_SERIAL_PROTOCOL_H
