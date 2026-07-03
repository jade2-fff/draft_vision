/**
 * @file dart_aimer.h
 * @brief 瞄准 — 取 target 的角度误差 + 射击判定（仅显示/录像，不发串口）
 */
#ifndef DART_AIMER_H
#define DART_AIMER_H

#include "dart_detector.h"

struct AimCommand {
    bool  valid = false;
    bool  fire  = false;     // 仅 HUD/录像用，不发串口
    float yaw_err   = 0;     // [度]
    float pitch_err = 0;     // [度]
};

class DartAimer {
public:
    DartAimer();

    /** 由已解算的 target 生成瞄准指令 */
    AimCommand aim(const DartTarget &target);

    void set_fire_tol(double yaw_tol, double pitch_tol) {
        fire_yaw_tol_   = yaw_tol;
        fire_pitch_tol_ = pitch_tol;
    }

private:
    double fire_yaw_tol_   = 1.0;   // 射击角度容差（度）
    double fire_pitch_tol_ = 1.0;
};

#endif
