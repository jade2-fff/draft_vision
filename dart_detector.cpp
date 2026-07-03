/**
 * @file dart_detector.cpp
 * @brief 绿色光源检测实现 — HSV + 亮核 + 亚像素强度质心 + confirmer
 */
#include "dart_detector.h"

#include "dart_confirmer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

DartDetector::DartDetector() {
    std::cout << "[Detector] HSV(" << lowH_ << "," << highH_
              << ") area=" << min_area_ << "-" << max_area_
              << " circ>" << min_circ_
              << " core(S>" << core_s_min_ << ",V>" << core_v_min_ << ")" << std::endl;
    kernel_open_  = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
    kernel_close_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
    confirmer_    = std::make_unique<DartConfirmer>();
}

void DartDetector::set_hsv(int lH, int hH, int lS, int hS, int lV, int hV) {
    lowH_=lH; highH_=hH; lowS_=lS; highS_=hS; lowV_=lV; highV_=hV;
}

DartTarget DartDetector::detect_raw(const cv::Mat &bgr) {
    DartTarget target;
    if (bgr.empty()) return target;

    // 1. HSV 阈值
    cv::cvtColor(bgr, hsv_, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_, cv::Scalar(lowH_, lowS_, lowV_),
                     cv::Scalar(highH_, highS_, highV_), mask_);

    // 2. 形态学去噪
    cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, kernel_open_);
    cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, kernel_close_);
    last_mask_ = mask_.clone();

    // 3. 轮廓提取 + 过滤（面积/圆度/亮核）
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = 0.0;
    const std::vector<cv::Point> *best_contour = nullptr;
    cv::Point2f best_center{0.f, 0.f};
    float       best_radius = 0.f;

    for (const auto &c : contours) {
        double area = cv::contourArea(c);
        if (area < min_area_ || area > max_area_) continue;

        double perim = cv::arcLength(c, true);
        if (perim <= 0.0) continue;
        double circ = 4.0 * CV_PI * area / (perim * perim);
        if (circ < min_circ_) continue;

        cv::Point2f center;
        float radius = 0.f;
        cv::minEnclosingCircle(c, center, radius);

        // 亮核验证：中心区域 S/V 要够高（真发光体 vs 反光）
        int r = std::max(2, static_cast<int>(radius * 0.5f));
        cv::Rect core(static_cast<int>(center.x) - r, static_cast<int>(center.y) - r, 2*r, 2*r);
        core &= cv::Rect(0, 0, hsv_.cols, hsv_.rows);
        if (core.area() <= 0) continue;
        cv::Scalar core_mean = cv::mean(hsv_(core));
        if (core_mean[1] < core_s_min_ || core_mean[2] < core_v_min_) continue;

        double score = circ * area;   // 圆度 × 面积
        if (score > best_score) {
            best_score   = score;
            best_contour = &c;
            best_center  = center;
            best_radius  = radius;
        }
    }

    if (!best_contour) return target;

    target.radius = best_radius;

    // 4. 亚像素强度加权质心（CoG）：减背景基座，剔除饱和平顶像素
    cv::Rect bbox = cv::boundingRect(*best_contour);
    int margin = std::max(1, static_cast<int>(std::lround(best_radius)));
    cv::Rect window(bbox.x - margin, bbox.y - margin,
                    bbox.width + 2*margin, bbox.height + 2*margin);
    window &= cv::Rect(0, 0, bgr.cols, bgr.rows);

    bool localized = false;
    double cx = 0.0, cy = 0.0, confidence = 0.0;

    if (window.area() > 0) {
        cv::cvtColor(bgr(window), gray_, cv::COLOR_BGR2GRAY);

        // 背景基座 = 窗口边缘环均值，其 std 作噪声估计
        const int ring = std::min({2, gray_.rows, gray_.cols});
        double bsum = 0.0, bsum2 = 0.0;
        long bn = 0;
        for (int yy = 0; yy < gray_.rows; ++yy) {
            bool y_edge = (yy < ring) || (yy >= gray_.rows - ring);
            const uchar *row = gray_.ptr<uchar>(yy);
            for (int xx = 0; xx < gray_.cols; ++xx) {
                if (!(y_edge || xx < ring || xx >= gray_.cols - ring)) continue;
                double v = row[xx];
                bsum += v; bsum2 += v*v; ++bn;
            }
        }
        double bg = (bn > 0) ? bsum / double(bn) : 0.0;
        double bnoise = (bn > 0) ? std::sqrt(std::max(0.0, bsum2/double(bn) - bg*bg)) : 0.0;

        // 加权质心 w = max(0, I - bg)，剔除 I=255 饱和平顶
        constexpr int kSaturation = 255;
        double sw = 0.0, swx = 0.0, swy = 0.0, peak = 0.0;
        for (int yy = 0; yy < gray_.rows; ++yy) {
            const uchar *row = gray_.ptr<uchar>(yy);
            double view_y = double(window.y + yy);
            for (int xx = 0; xx < gray_.cols; ++xx) {
                int I = row[xx];
                if (I >= kSaturation) continue;
                double w = double(I) - bg;
                if (w <= 0.0) continue;
                sw += w;
                swx += w * double(window.x + xx);
                swy += w * view_y;
                if (w > peak) peak = w;
            }
        }
        if (sw > 0.0) {
            cx = swx / sw;
            cy = swy / sw;
            // 置信度 = peak/噪声 SNR（定位质量），映射到 [0,1]
            constexpr double kNoiseFloor = 1.0;
            constexpr double kSnrHalf    = 5.0;
            double snr = peak / std::max(bnoise, kNoiseFloor);
            confidence = snr / (snr + kSnrHalf);
            localized = true;
        }
    }

    if (localized) {
        target.center     = cv::Point2f(float(cx), float(cy));
        target.bbox       = bbox;
        target.confidence = float(std::clamp(confidence, 0.0, 1.0));
    } else {
        // 退化回退：用 minEnclosingCircle 中心，保证不返回垃圾
        target.center     = best_center;
        target.bbox       = bbox;
        target.confidence = 0.f;
    }
    return target;
}

DartTarget DartDetector::detect(const cv::Mat &bgr) {
    DartTarget raw = detect_raw(bgr);
    if (raw.confidence > 0.f) raw.found = true;
    return confirmer_->update(raw);
}

int DartDetector::state() const {
    return confirmer_ ? int(confirmer_->state()) : 0;
}
