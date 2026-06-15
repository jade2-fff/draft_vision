#include "tasks/dart_auto_aim/detector/dart_detector.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace dart_auto_aim
{

DartDetector::DartDetector(const std::string & config_path)
{
  YAML::Node config = YAML::LoadFile(config_path);

  if (config["dart_detector"]) {
    auto dc = config["dart_detector"];

    // HSV 参数
    if (dc["HSV"]) {
      lowH_  = dc["HSV"]["lowH"].as<int>(lowH_);
      highH_ = dc["HSV"]["highH"].as<int>(highH_);
      lowS_  = dc["HSV"]["lowS"].as<int>(lowS_);
      highS_ = dc["HSV"]["highS"].as<int>(highS_);
      lowV_  = dc["HSV"]["lowV"].as<int>(lowV_);
      highV_ = dc["HSV"]["highV"].as<int>(highV_);
    }

    // 轮廓过滤
    if (dc["contours"]) {
      min_area_         = dc["contours"]["min_area"].as<double>(min_area_);
      max_area_         = dc["contours"]["max_area"].as<double>(max_area_);
      min_circularity_  = dc["contours"]["min_circularity"].as<double>(min_circularity_);
      min_fill_ratio_   = dc["contours"]["min_fill_ratio"].as<double>(min_fill_ratio_);
      min_aspect_ratio_ = dc["contours"]["min_aspect_ratio"].as<double>(min_aspect_ratio_);
    }

    debug_gui_ = dc["gui"].as<bool>(false);
  }

  tools::logger()->info("[DartDetector] Init: HSV({},{} {},{}, {},{}) area=[{:.0f},{:.0f}]",
                        lowH_, highH_, lowS_, highS_, lowV_, highV_, min_area_, max_area_);
}

void DartDetector::set_hsv_range(int lowH, int highH, int lowS, int highS, int lowV, int highV)
{
  lowH_ = lowH;  highH_ = highH;
  lowS_ = lowS;  highS_ = highS;
  lowV_ = lowV;  highV_ = highV;
}

void DartDetector::set_filter_params(double min_area, double max_area, double min_c, double min_fill)
{
  min_area_ = min_area;
  max_area_ = max_area;
  min_circularity_ = min_c;
  min_fill_ratio_ = min_fill;
}

std::vector<DartTarget> DartDetector::detect(
  const cv::Mat & bgr,
  std::chrono::steady_clock::time_point timestamp,
  int frame_id)
{
  // YOLO 回调优先
  if (yolo_callback_) {
    return yolo_callback_(bgr, timestamp, frame_id);
  }

  return detect_traditional(bgr, timestamp, frame_id);
}

std::vector<DartTarget> DartDetector::detect_traditional(
  const cv::Mat & bgr,
  std::chrono::steady_clock::time_point timestamp,
  int frame_id)
{
  std::vector<DartTarget> results;

  if (bgr.empty()) return results;

  // ── 1. BGR → HSV ──
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  // ── 2. 颜色阈值 ──
  cv::Scalar lower(lowH_, lowS_, lowV_);
  cv::Scalar upper(highH_, highS_, highV_);
  cv::Mat mask;
  cv::inRange(hsv, lower, upper, mask);

  // ── 3. 形态学去噪 ──
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  // ── 4. 轮廓提取 ──
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // ── 5. 轮廓过滤 + 生成候选 ──
  for (size_t i = 0; i < contours.size(); ++i) {
    const auto & c = contours[i];

    double area = cv::contourArea(c);
    double perimeter = cv::arcLength(c, true);
    if (perimeter < 1e-6) continue;

    // 圆度
    double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);

    // 最小外接旋转矩形
    cv::RotatedRect rrect = cv::minAreaRect(c);
    double w = rrect.size.width;
    double h = rrect.size.height;
    if (w <= 0 || h <= 0) continue;

    double rect_area = w * h;
    double fill_ratio = area / rect_area;
    double aspect_ratio = std::min(w, h) / std::max(w, h);

    // ── 过滤条件 ──
    if (area < min_area_ || area > max_area_) continue;
    if (fill_ratio < min_fill_ratio_) continue;
    if (aspect_ratio < min_aspect_ratio_) continue;
    if (circularity < min_circularity_) continue;

    // 最小外接圆
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(c, center, radius);

    // 边界框
    cv::Rect bbox = cv::boundingRect(c);

    // 构建 DartTarget
    DartTarget t;
    t.center       = center;
    t.radius       = radius;
    t.bbox         = bbox;
    t.confidence   = static_cast<float>(std::min(circularity, 1.0));
    t.timestamp    = timestamp;
    t.image_size   = cv::Size(bgr.cols, bgr.rows);

    results.push_back(std::move(t));
  }

  // 按置信度排序，取最高若干
  std::sort(results.begin(), results.end(),
            [](const DartTarget & a, const DartTarget & b) {
              return a.confidence > b.confidence;
            });

  // 最多保留 3 个候选
  if (results.size() > 3) {
    results.resize(3);
  }

  // ── 调试窗口 ──
  if (debug_gui_) {
    static auto last_dbg = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double, std::milli>(now - last_dbg).count();
    if (dt > 33.3) {  // ~30 Hz
      cv::imshow("[Dart] mask", mask);
      cv::Mat dbg = bgr.clone();
      for (auto & t : results) {
        cv::circle(dbg, t.center, static_cast<int>(t.radius), cv::Scalar(0, 255, 0), 2);
        cv::circle(dbg, t.center, 3, cv::Scalar(0, 0, 255), -1);
        cv::rectangle(dbg, t.bbox, cv::Scalar(0, 255, 255), 2);
      }
      cv::imshow("[Dart] detect", dbg);
      cv::waitKey(1);
      last_dbg = now;
    }
  }

  return results;
}

}  // namespace dart_auto_aim
