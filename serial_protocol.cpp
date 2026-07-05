/**
 * @file serial_protocol.cpp
 * @brief 串口 + CRC16 协议层实现（与下位机 Gimbal 协议完全一致）
 */
#include "serial_protocol.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <unordered_map>
#include <vector>

// ── 串口包录制钩子 ──
static SerialRecorderHooks g_recorder_hooks;
void serial_set_recorder_hooks(const SerialRecorderHooks &hooks) {
    g_recorder_hooks = hooks;
}

// ═══════════════════════════════════════════
// CRC16 查表法（多项式 0x1189，初始值 0xFFFF，右移算法）
// 与下位机 / bof_26_vision 框架完全一致
// ═══════════════════════════════════════════
static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t crc16_calculate(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        uint8_t i = (crc ^ *data++) & 0x00ff;
        crc = (crc >> 8) ^ CRC16_TABLE[i];
    }
    return crc;
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

// ═══════════════════════════════════════════
// 发送 VisionToGimbal
// ═══════════════════════════════════════════

void serial_send_gimbal_cmd(int fd, uint8_t mode, float yaw, float pitch) {
    if (fd < 0) return;

    VisionToGimbal tx;
    tx.head     = 0x50;
    tx.mode     = mode;
    tx.yaw      = yaw;
    tx.yaw_vel  = 0;
    tx.yaw_acc  = 0;
    tx.pitch    = pitch;
    tx.pitch_vel = 0;
    tx.pitch_acc = 0;
    tx.crc16    = crc16_calculate(reinterpret_cast<uint8_t *>(&tx),
                                  sizeof(tx) - sizeof(tx.crc16));

    // 短写重试
    size_t sent = 0;
    const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&tx);
    while (sent < sizeof(tx)) {
        ssize_t n = write(fd, ptr + sent, sizeof(tx) - sent);
        if (n <= 0) break;
        sent += size_t(n);
    }

    if (g_recorder_hooks.on_tx) g_recorder_hooks.on_tx(ptr, sizeof(tx));
}

// ═══════════════════════════════════════════
// 接收 GimbalToVision（非阻塞，扫描 0x53 帧头）
// ═══════════════════════════════════════════

static std::unordered_map<int, std::vector<uint8_t>> g_rx_buf;

bool serial_recv_gimbal_state(int fd, float &yaw, float &pitch) {
    if (fd < 0) return false;

    auto &buf = g_rx_buf[fd];

    // 非阻塞读，追加到环形缓冲
    uint8_t tmp[256];
    ssize_t n;
    while ((n = read(fd, tmp, sizeof(tmp))) > 0) {
        buf.insert(buf.end(), tmp, tmp + n);
    }

    // 防爆：缓冲太长截断
    if (buf.size() > 512) buf.clear();

    // 扫描 0x53 帧头
    while (buf.size() >= sizeof(GimbalToVision)) {
        if (buf[0] != 0x53) {
            buf.erase(buf.begin());  // 不是帧头，丢掉继续找
            continue;
        }

        // 尝试 CRC 校验
        const GimbalToVision *rx = reinterpret_cast<const GimbalToVision *>(buf.data());
        uint16_t computed = crc16_calculate(buf.data(), sizeof(GimbalToVision) - sizeof(rx->crc16));
        uint16_t received = rx->crc16;

        if (computed == received) {
            yaw   = rx->yaw;
            pitch = rx->pitch;

            if (g_recorder_hooks.on_rx) g_recorder_hooks.on_rx(buf.data(), sizeof(GimbalToVision));
            buf.erase(buf.begin(), buf.begin() + sizeof(GimbalToVision));
            return true;
        }

        // CRC 不对，丢掉帧头字节继续找
        buf.erase(buf.begin());
    }

    return false;
}
