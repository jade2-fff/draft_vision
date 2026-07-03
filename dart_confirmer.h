/**
 * @file dart_confirmer.h
 * @brief 跟踪确认状态机 — 移植自 master2 dart_confirmer
 *
 * 三态: SEARCHING → CANDIDATE → LOCKED
 * - 连续 confirm_frames 帧一致才锁定
 * - 帧间跳变 > max_jump_px 视为不一致（防误跟）
 * - miss_frames 帧丢失才解锁
 * - LOCKED 后运动自适应 EMA 融合：稳定时强平滑压亚像素噪声，转动时无滞后
 */
#ifndef DART_CONFIRMER_H
#define DART_CONFIRMER_H

#include "dart_detector.h"

class DartConfirmer {
public:
    enum State { SEARCHING = 0, CANDIDATE = 1, LOCKED = 2 };

    DartConfirmer(int confirm_frames = 3, int miss_frames = 8, double max_jump_px = 30.0);

    void reset();
    State state() const { return state_; }

    /** 输入单帧 raw 检测，输出确认后的目标（未锁定时 found=false） */
    DartTarget update(const DartTarget &raw);

private:
    int    confirm_frames_;
    int    miss_frames_;
    double max_jump_px_;

    State       state_       = SEARCHING;
    int         hit_count_   = 0;
    int         miss_count_  = 0;
    DartTarget  last_;
    DartTarget  fused_;
};

#endif
