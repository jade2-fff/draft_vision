/**
 * @file serial_protocol.h
 * @brief 串口 + CRC16 协议层 — 与下位机 USB_usart 协议完全一致
 *
 * 帧格式: [12字节 payload (3×int32_t, 值×100)] + [2字节 CRC16]
 *   CRC16: 多项式 0x1021, 初值 0xFFFF, 左移查表法
 *   CRC 字节序: 低字节在前, 高字节在后
 *   波特率: 115200 8N1
 *
 * 字段语义:
 *   上位机→下位机: [0]=x [4]=y [8]=z
 *       x/y = 目标平面物理距离(dx/dy)，z = valid
 *   下位机→上位机: [0]=pitch [4]=yaw [8]=roll （度）
 */
#ifndef DART_SERIAL_PROTOCOL_H
#define DART_SERIAL_PROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <termios.h>

// ── 协议常量（与下位机 USB_usart.h 一致） ──
#define DATA_PAYLOAD_SIZE  12
#define CRC_SIZE           2
#define TOTAL_PACKET_SIZE  (DATA_PAYLOAD_SIZE + CRC_SIZE)  // 14字节

// ── 串口 ──
int  serial_open(const char *port, int baud);
void serial_close(int fd);

// ── CRC16（多项式 0x1021，左移查表，与下位机一致） ──
uint16_t crc16_calculate(const uint8_t *data, uint16_t length);

// ── float → int32(×100) ──
// clamp_max 默认放大到可容纳深度(如 25000mm)；int32/100 上限约 2100 万
int32_t float_to_int100(float val, int32_t clamp_max = 20000000);

/** 上位机→下位机：发 x/y/z（值×100 转 int32），带短写重试 */
void serial_send_packet(int fd, float v1, float v2, float v3);

/** 通用接收：环形缓冲 + CRC 滑动对齐，解析 3 个 float */
bool serial_recv_packet(int fd, float &v1, float &v2, float &v3);

// ── 语义化包装 ──
/** 上位机→下位机：发目标平面偏差 dx/dy + valid */
inline void serial_send_plane_offset(int fd, float dx, float dy, float valid) {
    serial_send_packet(fd, dx, dy, valid);
}
/** 下位机→上位机：收 pitch/yaw/roll（度），按下位机字段顺序 */
inline bool serial_recv_attitude(int fd, float &pitch, float &yaw, float &roll) {
    return serial_recv_packet(fd, pitch, yaw, roll);
}

// ── 串口原始包录制钩子（由 main 注入，默认 nullptr 不录） ──
struct SerialRecorderHooks {
    void (*on_tx)(const uint8_t *data, size_t size) = nullptr;
    void (*on_rx)(const uint8_t *data, size_t size) = nullptr;
};
void serial_set_recorder_hooks(const SerialRecorderHooks &hooks);

#endif // DART_SERIAL_PROTOCOL_H
