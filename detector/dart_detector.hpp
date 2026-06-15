/**
 * @file dart_detector.hpp
 * @brief 飞镖靶检测器 — HSV 颜色阈值 + 圆形检测，预留 YOLO 接口
 *
 * 检测流程:
 *   1. BGR → HSV 颜色空间转换
 *   2. 绿色阈值 inRange (飞镖靶为绿色圆形区域)
 *   3. 形态学开/闭运算去噪
 *   4. 轮廓提取 + 圆度/填充率/面积/宽高比过滤
 *   5. 输出 DartTarget 列表
 *
 * 预留接口: set_yolo_detector() 可接入 YOLOv5/v8/v11 神经网络检测
 */

#ifndef DART_AUTO_AIM__DART_DETECTOR_HPP
#define DART_AUTO_AIM__DART_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "tasks/dart_auto_aim/common/dart_types.hpp"

namespace dart_auto_aim
{

/**
 * @class DartDetector
 * @brief 飞镖靶传统视觉检测器 (HSV 颜色分割)
 *
 * 使用方式:
 * @code
 *   DartDetector detector("configs/dart.yaml");
 *   auto targets = detector.detect(bgr_image, timestamp, frame_id);
 * @endcode
 */
class DartDetector
{
public:
  /** @brief 从 YAML 配置构造 */
  explicit DartDetector(const std::string & config_path);

  /**
   * @brief 检测一帧中的所有飞镖靶
   * @param bgr       BGR 格式输入图像
   * @param timestamp 时间戳
   * @param frame_id  帧序号
   * @return 检测到的飞镖靶列表
   */
  std::vector<DartTarget> detect(
    const cv::Mat & bgr,
    std::chrono::steady_clock::time_point timestamp,
    int frame_id = 0);

  // ── 预留 YOLO 接口 ──
  /** @brief 设置 YOLO 回调 (未来替换传统CV) */
  using YoloCallback = std::function<std::vector<DartTarget>(
    const cv::Mat &, std::chrono::steady_clock::time_point, int)>;
  void set_yolo_callback(YoloCallback cb) { yolo_callback_ = std::move(cb); }
  bool use_yolo() const { return static_cast<bool>(yolo_callback_); }

  // ── 配置 setter (调试用) ──
  void set_hsv_range(int lowH, int highH, int lowS, int highS, int lowV, int highV);
  void set_filter_params(double min_area, double max_area, double min_circularity, double min_fill);

private:
  // ── HSV 阈值 ──
  int lowH_  = 25,  highH_ = 85;
  int lowS_  = 50,  highS_ = 255;
  int lowV_  = 80,  highV_ = 255;

  // ── 轮廓过滤参数 ──
  double min_area_         = 100;    ///< 最小面积 [px²]
  double max_area_         = 50000;  ///< 最大面积
  double min_circularity_  = 0.5;    ///< 最小圆度 (4πA/P²)
  double min_fill_ratio_   = 0.7;    ///< 最小填充率 (A/(w*h))
  double min_aspect_ratio_ = 0.7;    ///< 最小宽高比 (min/max)

  // ── 调试 ──
  bool debug_gui_ = false;

  // ── YOLO 预留 ──
  YoloCallback yolo_callback_;

  /** @brief 内部: 传统 CV 检测 */
  std::vector<DartTarget> detect_traditional(
    const cv::Mat & bgr,
    std::chrono::steady_clock::time_point timestamp,
    int frame_id);
};

}  // namespace dart_auto_aim

#endif  // DART_AUTO_AIM__DART_DETECTOR_HPP
