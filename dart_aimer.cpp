/**
 * @file dart_aimer.cpp
 * @brief 瞄准实现 — 取角度误差 + 射击判定
 */
#include "dart_aimer.h"

#include <cmath>
#include <iostream>

DartAimer::DartAimer() {
    std::cout << "[Aimer] fire_tol=(" << fire_yaw_tol_
              << "," << fire_pitch_tol_ << ") deg" << std::endl;
}

AimCommand DartAimer::aim(const DartTarget &target) {
    AimCommand cmd;
    if (!target.found) return cmd;

    cmd.valid     = true;
    cmd.yaw_err   = target.yaw_err;
    cmd.pitch_err = target.pitch_err;
    // 仅显示用：角度误差在容差内即标记可开火（非火控逻辑）
    cmd.fire = std::abs(target.yaw_err)   < fire_yaw_tol_ &&
               std::abs(target.pitch_err) < fire_pitch_tol_;
    return cmd;
}
