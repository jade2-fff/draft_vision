/**
 * @file dart_confirmer.cpp
 * @brief 跟踪确认状态机实现
 */
#include "dart_confirmer.h"

#include <algorithm>
#include <cmath>

static double dist(const cv::Point2f &a, const cv::Point2f &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

DartConfirmer::DartConfirmer(int confirm_frames, int miss_frames, double max_jump_px)
    : confirm_frames_(confirm_frames), miss_frames_(miss_frames), max_jump_px_(max_jump_px) {}

void DartConfirmer::reset() {
    state_      = SEARCHING;
    hit_count_  = 0;
    miss_count_ = 0;
    last_       = DartTarget{};
    fused_      = DartTarget{};
}

DartTarget DartConfirmer::update(const DartTarget &raw) {
    // raw.confidence>0 视为本帧检测到
    bool raw_found = raw.confidence > 0.f || raw.found;
    bool consistent = raw_found && dist(raw.center, last_.center) <= max_jump_px_;

    switch (state_) {
        case SEARCHING:
            if (raw_found) {
                state_     = CANDIDATE;
                hit_count_ = 1;
                miss_count_ = 0;
                last_      = raw;
            }
            break;

        case CANDIDATE:
            if (raw_found && consistent) {
                hit_count_++;
                miss_count_ = 0;
                last_ = raw;
                if (hit_count_ >= confirm_frames_) {
                    state_ = LOCKED;
                    fused_ = last_;
                }
            } else if (raw_found) {
                hit_count_ = 1;
                miss_count_ = 0;
                last_ = raw;
            } else if (++miss_count_ >= miss_frames_) {
                reset();
            }
            break;

        case LOCKED: {
            if (raw_found && consistent) {
                // 运动自适应 EMA：d 小（稳定）→ alpha 小强平滑；d 大（转动）→ alpha→1 无滞后
                static constexpr double kTrackPx  = 3.0;
                static constexpr double kAlphaMin = 0.15;

                double d     = dist(raw.center, last_.center);
                double alpha = std::clamp(d / kTrackPx, kAlphaMin, 1.0);
                double beta  = 1.0 - alpha;

                fused_.center.x    = float(beta * fused_.center.x + alpha * raw.center.x);
                fused_.center.y    = float(beta * fused_.center.y + alpha * raw.center.y);
                fused_.radius      = float(beta * fused_.radius + alpha * raw.radius);
                fused_.confidence  = float(beta * fused_.confidence + alpha * raw.confidence);

                last_ = raw;
                miss_count_ = 0;
            } else if (++miss_count_ >= miss_frames_) {
                reset();
            }
            break;
        }
    }

    DartTarget out;
    if (state_ == LOCKED) {
        out = fused_;
        out.found = true;
    }
    return out;
}
