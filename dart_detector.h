/**
 * @file dart_detector.h
 * @brief 绿色光源检测 — HSV + 亮核验证 + 亚像素强度质心 + 跟踪确认
 *
 * 移植自 master2 vision/green_light_detector，零框架依赖。
 * 管线: HSV阈值 → 形态学 → 轮廓过滤(面积/圆度/亮核) → 亚像素质心 → confirmer确认
 */
#ifndef DART_DETECTOR_H
#define DART_DETECTOR_H

#include <memory>

#include <opencv2/opencv.hpp>

class DartConfirmer;

struct DartTarget {
    bool        found = false;       // 经 confirmer 确认
    cv::Point2f center{0.f, 0.f};    // 像素中心（亚像素质心）
    float       radius = 0;          // 光斑半径（像素）
    cv::Rect    bbox;
    float       confidence = 0;      // SNR 置信度 [0,1]

    // solver 填充：相对镗准线的角度误差（度）
    float yaw_err   = 0;   // 右偏为正
    float pitch_err = 0;   // 上偏为正

    // solver 填充：射线与目标平面 Z=0 的交点/偏差（mm）
    cv::Point2f plane_point_mm{0.f, 0.f};   // 目标点在目标平面坐标系中的绝对坐标
    cv::Point2f plane_offset_mm{0.f, 0.f};  // 相对镗准线交点的平面偏差，右/上为正
    bool        plane_valid = false;        // 外参有效且求交成功

    // solver 填充：目标点在相机坐标系下的坐标（原点=相机/发射点，mm）
    float cam_x_mm     = 0.f;   // 水平偏移，右为正
    float cam_depth_mm = 0.f;   // 前向深度（相机 Z 轴）

    /** 归一化中心偏移 [-1,1]（保留给旧调用方/调试） */
    float cx_norm(int imgW) const { return imgW>0 ? center.x/float(imgW)*2.f-1.f : 0.f; }
    float cy_norm(int imgH) const { return imgH>0 ? center.y/float(imgH)*2.f-1.f : 0.f; }
};

class DartDetector {
public:
    DartDetector();

    /** HSV 阈值 (默认绿: H35-85, S50-255, V50-255) */
    void set_hsv(int lH, int hH, int lS, int hS, int lV, int hV);

    /** 检测一帧（含 confirmer 确认），返回目标 */
    DartTarget detect(const cv::Mat &bgr);

    /** confirmer 当前状态（SEARCHING/CANDIDATE/LOCKED） */
    int state() const;

    /** 最近一次 HSV mask（可视化/调试用） */
    const cv::Mat &debug_mask() const { return last_mask_; }

private:
    DartTarget detect_raw(const cv::Mat &bgr);

    // HSV 阈值
    int lowH_=35, highH_=85, lowS_=50, highS_=255, lowV_=50, highV_=255;

    // 过滤参数
    double min_area_     = 8.0;
    double max_area_     = 8000.0;
    double min_circ_     = 0.6;
    double core_s_min_   = 120;   // 亮核最低饱和度
    double core_v_min_   = 200;   // 亮核最低亮度

    cv::Mat last_mask_;
    cv::Mat hsv_, mask_, gray_;
    cv::Mat kernel_open_, kernel_close_;

    std::unique_ptr<DartConfirmer> confirmer_;
};

#endif
