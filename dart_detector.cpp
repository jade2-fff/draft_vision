/**
 * @file dart_detector.cpp
 * @brief 绿色光源检测实现
 */
#include "dart_detector.h"
#include <cmath>
#include <iostream>

DartDetector::DartDetector() {
    std::cout << "[Detector] HSV(" << lowH_ << "," << highH_
              << ") area=" << min_area_ << "-" << max_area_
              << " circ>" << min_circ_ << std::endl;
}

void DartDetector::set_hsv(int lH, int hH, int lS, int hS, int lV, int hV) {
    lowH_=lH; highH_=hH; lowS_=lS; highS_=hS; lowV_=lV; highV_=hV;
}

DartTarget DartDetector::detect(const cv::Mat &bgr) {
    DartTarget best{};

    if (bgr.empty()) return best;

    // 1. HSV 阈值
    cv::Mat hsv, mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(lowH_, lowS_, lowV_),
                     cv::Scalar(highH_, highS_, highV_), mask);

    // 2. 形态学去噪
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5}));

    // 3. 轮廓提取 + 过滤
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double bestConf = 0;
    for (auto &c : contours) {
        double area = cv::contourArea(c);
        if (area < min_area_ || area > max_area_) continue;

        // 圆形度
        double perim = cv::arcLength(c, true);
        double circ = (4.0 * CV_PI * area) / (perim * perim);
        if (circ < min_circ_) continue;

        cv::RotatedRect rr = cv::minAreaRect(c);
        DartTarget t;
        t.center     = rr.center;
        t.radius     = float(rr.size.width + rr.size.height) * 0.25f;
        t.bbox       = cv::boundingRect(c);
        t.confidence = float(std::min(circ, 1.0));

        if (t.confidence > bestConf) {
            bestConf = t.confidence;
            best = t;
        }
    }

    return best;
}
