/**
 * @file serial_protocol.h
 * @brief 串口 + CRC16 协议层
 *
 * 与下位机 USB_usart 协议完全一致:
 *   帧格式: [12字节 payload (3×int32_t, 值×100)] + [2字节 CRC16 (多项式 0x1021)]
 *   波特率: 115200 8N1
 *
 * 字段语义（飞镖单下位机，无云台）:
 *   上位机→下位机: [0]=plane_dx_mm [4]=plane_dy_mm [8]=valid
 *       dx/dy 为绿灯平面交点相对镗准线平面交点的距离，右/上为正，单位 mm
 *   下位机→上位机: [0]=pitch [4]=yaw [8]=roll （姿态，度）
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

// ── CRC16 ──
uint16_t crc16_calculate(const uint8_t *data, uint16_t length);

// ── 组包/收发（通用，按位置传 3 个 float） ──
void packet_build(const uint8_t payload[DATA_PAYLOAD_SIZE],
                  uint8_t       packet[TOTAL_PACKET_SIZE]);

int32_t float_to_int100(float val, int32_t clamp_max = 99999);

/** 上位机→下位机：发 x/y/z（值×100 转 int32），带短写重试 */
void serial_send_packet(int fd, float v1, float v2, float v3);

/** 通用接收：环形缓冲 + CRC 滑动对齐，解析 3 个 float */
bool serial_recv_packet(int fd, float &v1, float &v2, float &v3);

// ── 语义化包装 ──
inline void serial_send_xyz(int fd, float x, float y, float z) {
    serial_send_packet(fd, x, y, z);
}
/** 上位机→下位机：发目标平面偏差 dx/dy(mm) + valid */
inline void serial_send_plane_offset(int fd, float dx_mm, float dy_mm, float valid) {
    serial_send_packet(fd, dx_mm, dy_mm, valid);
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
