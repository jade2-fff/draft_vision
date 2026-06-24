/**
 * @file dart_aimer.cpp
 * @brief 瞄准实现 — 像素偏移 → 角度
 */
#include "dart_aimer.h"
#include <cmath>
#include <iostream>

DartAimer::DartAimer() {
    std::cout << "[Aimer] yaw_scale=" << yaw_s_
              << " pitch_scale=" << pitch_s_
              << " fire_tol=(" << fire_x_ << "," << fire_y_ << ")" << std::endl;
}

AimCommand DartAimer::aim(float cx_norm, float cy_norm, bool detected) const {
    AimCommand cmd;
    if (!detected) return cmd;

    cmd.valid = true;
    cmd.yaw   = float(-cx_norm * yaw_s_);
    cmd.pitch = float(-cy_norm * pitch_s_);
    cmd.fire  = std::abs(cx_norm) < fire_x_ && std::abs(cy_norm) < fire_y_;

    return cmd;
}
