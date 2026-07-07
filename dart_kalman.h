/**
 * @file dart_kalman.h
 * @brief 1D 卡尔曼滤波 — 常速模型(CV)，平滑角度序列去抖
 *
 * 状态 x = [pos, vel]，测量只有 pos。用于平滑 solver 的 yaw_err。
 * 固定靶只平滑不预测，输出滤波后的 pos。零框架依赖，纯头文件。
 */
#ifndef DART_KALMAN_H
#define DART_KALMAN_H

#include <cmath>

class Kalman1D {
public:
    /**
     * @param q 过程噪声（越大越信任测量，跟随快但抖；越小越平滑但滞后）
     * @param r 测量噪声（越大越平滑，越小越跟随）
     */
    explicit Kalman1D(double q = 1.0, double r = 25.0) : q_(q), r_(r) {}

    void set_noise(double q, double r) { q_ = q; r_ = r; }

    /** 无目标/目标跳变时重置，下次从新测量重新起滤 */
    void reset() { initialized_ = false; }

    /**
     * 用新测量更新，返回平滑后的位置
     * @param meas 本帧测量值（如 yaw_err，度）
     * @param dt   距上帧时间(秒)，用于常速预测
     */
    double update(double meas, double dt) {
        if (!initialized_) {
            x_ = meas; v_ = 0.0;
            p00_ = 1.0; p01_ = 0.0; p10_ = 0.0; p11_ = 1.0;
            initialized_ = true;
            return x_;
        }
        if (dt <= 0.0 || dt > 1.0) dt = 0.033;   // 异常 dt 兜底 ~30fps

        // 预测：x = x + v*dt
        x_ += v_ * dt;
        // P = F P Fᵀ + Q
        double dt2 = dt * dt;
        double p00 = p00_ + dt * (p10_ + p01_) + dt2 * p11_ + q_ * dt2 * dt2 / 4.0;
        double p01 = p01_ + dt * p11_ + q_ * dt2 * dt / 2.0;
        double p10 = p10_ + dt * p11_ + q_ * dt2 * dt / 2.0;
        double p11 = p11_ + q_ * dt2;
        p00_ = p00; p01_ = p01; p10_ = p10; p11_ = p11;

        // 更新：测量 z = pos
        double s = p00_ + r_;
        double k0 = p00_ / s;
        double k1 = p10_ / s;
        double y = meas - x_;
        x_ += k0 * y;
        v_ += k1 * y;

        double np00 = (1.0 - k0) * p00_;
        double np01 = (1.0 - k0) * p01_;
        double np10 = p10_ - k1 * p00_;
        double np11 = p11_ - k1 * p01_;
        p00_ = np00; p01_ = np01; p10_ = np10; p11_ = np11;

        return x_;
    }

    double value() const { return x_; }
    bool ready() const { return initialized_; }

private:
    double q_, r_;
    double x_ = 0.0, v_ = 0.0;
    double p00_ = 1.0, p01_ = 0.0, p10_ = 0.0, p11_ = 1.0;
    bool initialized_ = false;
};

#endif // DART_KALMAN_H
