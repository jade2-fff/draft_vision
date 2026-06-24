/**
 * @file dart_detector.h
 * @brief 绿色光源检测 — HSV 阈值 + 简单轮廓过滤
 */
#ifndef DART_DETECTOR_H
#define DART_DETECTOR_H

#include <opencv2/opencv.hpp>

struct DartTarget {
    cv::Point2f center;
    float       radius = 0;
    cv::Rect    bbox;
    float       confidence = 0;

    /** 归一化中心偏移 [-1, 1]，0=画面中心 */
    float cx_norm(int imgW) const { return imgW>0 ? center.x/float(imgW)*2.f-1.f : 0.f; }
    float cy_norm(int imgH) const { return imgH>0 ? center.y/float(imgH)*2.f-1.f : 0.f; }
};

class DartDetector {
public:
    DartDetector();

    /** HSV 阈值 (默认绿: H35-85, S50-255, V50-255) */
    void set_hsv(int lH, int hH, int lS, int hS, int lV, int hV);

    /** 检测一帧，返回最佳目标 (无目标时 confidence=0) */
    DartTarget detect(const cv::Mat &bgr);

private:
    int lowH_=35, highH_=85, lowS_=50, highS_=255, lowV_=50, highV_=255;

    // 过滤参数
    double min_area_ = 100;
    double max_area_ = 50000;
    double min_circ_ = 0.7;   // 最小圆形度
};

#endif
