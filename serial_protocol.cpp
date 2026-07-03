/**
 * @file serial_protocol.cpp
 * @brief 串口 + CRC16 协议层实现
 */
#include "serial_protocol.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstring>

// ── 串口包录制钩子（可选，由 main 注入） ──
static SerialRecorderHooks g_recorder_hooks;
void serial_set_recorder_hooks(const SerialRecorderHooks &hooks) {
    g_recorder_hooks = hooks;
}

// ═══════════════════════════════════════════
// CRC16 查表法（多项式 0x1021，初始值 0xFFFF）
// 直接复用下位机 USB_usart.c 的 CRC16_TABLE
// ═══════════════════════════════════════════
static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t crc16_calculate(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    while (length--) {
        uint8_t index = (crc >> 8) ^ *data++;
        crc = (crc << 8) ^ CRC16_TABLE[index];
    }
    return crc;
}

void packet_build(const uint8_t payload[DATA_PAYLOAD_SIZE],
                  uint8_t       packet[TOTAL_PACKET_SIZE]) {
    memcpy(packet, payload, DATA_PAYLOAD_SIZE);
    uint16_t crc = crc16_calculate(payload, DATA_PAYLOAD_SIZE);
    packet[DATA_PAYLOAD_SIZE]     = crc & 0xFF;
    packet[DATA_PAYLOAD_SIZE + 1] = (crc >> 8) & 0xFF;
}

int32_t float_to_int100(float val, int32_t clamp_max) {
    int32_t v = static_cast<int32_t>(std::round(val * 100.0));
    if      (v >  clamp_max) v =  clamp_max;
    else if (v < -clamp_max) v = -clamp_max;
    return v;
}

// ═══════════════════════════════════════════
// 串口
// ═══════════════════════════════════════════

int serial_open(const char *port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag |=  (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;   // 非阻塞读取
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    return fd;
}

void serial_close(int fd) {
    if (fd >= 0) close(fd);
}

void serial_send_packet(int fd, float v1, float v2, float v3) {
    if (fd < 0) return;

    uint8_t payload[DATA_PAYLOAD_SIZE];
    int32_t i1 = float_to_int100(v1);
    int32_t i2 = float_to_int100(v2);
    int32_t i3 = float_to_int100(v3);

    memcpy(&payload[0], &i1, sizeof(i1));
    memcpy(&payload[4], &i2, sizeof(i2));
    memcpy(&payload[8], &i3, sizeof(i3));

    uint8_t packet[TOTAL_PACKET_SIZE];
    packet_build(payload, packet);

    // 短写重试
    size_t sent = 0;
    while (sent < TOTAL_PACKET_SIZE) {
        ssize_t n = write(fd, packet + sent, TOTAL_PACKET_SIZE - sent);
        if (n <= 0) break;
        sent += size_t(n);
    }

    if (g_recorder_hooks.on_tx) g_recorder_hooks.on_tx(packet, TOTAL_PACKET_SIZE);
}

// 每个 fd 一个接收环形缓冲（CRC 滑动对齐用）
#include <unordered_map>
#include <vector>
static std::unordered_map<int, std::vector<uint8_t>> g_rx_buf;

bool serial_recv_packet(int fd, float &v1, float &v2, float &v3) {
    if (fd < 0) return false;

    auto &buf = g_rx_buf[fd];

    // 追加本次读到的新字节（非阻塞）
    uint8_t tmp[256];
    ssize_t n;
    while ((n = read(fd, tmp, sizeof(tmp))) > 0) {
        buf.insert(buf.end(), tmp, tmp + n);
    }

    // 滑动窗口找 14 字节使 CRC 通过的帧
    while (buf.size() >= TOTAL_PACKET_SIZE) {
        uint16_t recv_crc = buf[DATA_PAYLOAD_SIZE] | (buf[DATA_PAYLOAD_SIZE + 1] << 8);
        if (crc16_calculate(buf.data(), DATA_PAYLOAD_SIZE) == recv_crc) {
            int32_t i1, i2, i3;
            memcpy(&i1, &buf[0], sizeof(i1));
            memcpy(&i2, &buf[4], sizeof(i2));
            memcpy(&i3, &buf[8], sizeof(i3));
            v1 = i1 / 100.0f;
            v2 = i2 / 100.0f;
            v3 = i3 / 100.0f;

            if (g_recorder_hooks.on_rx) g_recorder_hooks.on_rx(buf.data(), TOTAL_PACKET_SIZE);
            buf.erase(buf.begin(), buf.begin() + TOTAL_PACKET_SIZE);
            return true;
        }
        buf.erase(buf.begin());   // 错位 1 字节继续找
    }

    // 防爆：缓冲过长（一直对不齐）截断
    if (buf.size() > 256) buf.clear();
    return false;
}
