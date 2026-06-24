/**
 * @file dart_aimer.h
 * @brief 瞄准 — 像素偏移 → 角度指令 + 射击判定
 */
#ifndef DART_AIMER_H
#define DART_AIMER_H

struct AimCommand {
    bool  valid = false;
    bool  fire  = false;
    float yaw   = 0;     // [rad]
    float pitch = 0;     // [rad]
};

class DartAimer {
public:
    DartAimer();

    /** 像素偏移 → 角度指令
     * @param cx_norm  归一化水平偏移 [-1, 1]
     * @param cy_norm  归一化垂直偏移 [-1, 1]
     * @param detected 是否检测到目标
     */
    AimCommand aim(float cx_norm, float cy_norm, bool detected) const;

    void set_scales(double yaw, double pitch)  { yaw_s_=yaw; pitch_s_=pitch; }
    void set_fire_tol(double x, double y)       { fire_x_=x; fire_y_=y; }

private:
    double yaw_s_   = 0.5;   // yaw 缩放
    double pitch_s_ = 0.3;   // pitch 缩放
    double fire_x_  = 0.05;  // 射击容差 (归一化)
    double fire_y_  = 0.05;
};

#endif
