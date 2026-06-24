/**
 * @file serial_protocol.h
 * @brief 串口 + CRC16 协议层
 *
 * 与下位机 USB_usart 协议完全一致:
 *   帧格式: [12字节 payload (3×int32_t, 值×100)] + [2字节 CRC16 (多项式 0x1021)]
 *   波特率: 115200 8N1
 */
#ifndef DART_SERIAL_PROTOCOL_H
#define DART_SERIAL_PROTOCOL_H

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

// ── 组包/收发 ──
void packet_build(const uint8_t payload[DATA_PAYLOAD_SIZE],
                  uint8_t       packet[TOTAL_PACKET_SIZE]);

int32_t float_to_int100(float val, int32_t clamp_max = 99999);

void serial_send_packet(int fd, float v1, float v2, float v3);
bool serial_recv_packet(int fd, float &v1, float &v2, float &v3);

#endif // DART_SERIAL_PROTOCOL_H
