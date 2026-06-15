/**
 * @file dart_types.hpp
 * @brief 飞镖靶数据结构 — 飞镖自瞄核心类型定义
 *
 * 飞镖靶是圆形靶面（不同于步兵装甲板的灯条结构），
 * 检测输出为圆中心+外接矩形，PnP 解算 3D 位姿后送入 EKF 跟踪。
 */

#ifndef DART_AUTO_AIM__DART_TYPES_HPP
#define DART_AUTO_AIM__DART_TYPES_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>

namespace dart_auto_aim
{

/**
 * @struct DartTarget
 * @brief 飞镖靶检测结果 — 单个圆形靶面的检测+解算数据
 *
 * 从检测器输出到 tracker，经过 solver 后填充 3D 位姿。
 * 飞镖靶是静态圆形靶（挂在墙上），世界坐标系下为竖直平面，
 * 靶面直径约 14cm，PnP 用外接矩形四角点解算。
 */
struct DartTarget
{
  // ── 2D 检测信息 ──
  cv::Point2f center;            ///< 靶心像素坐标
  float       radius;            ///< 靶半径 [px]
  cv::Rect    bbox;              ///< 外接矩形 (x, y, w, h)
  float       confidence;        ///< 检测置信度 [0, 1]

  // ── PnP 解算结果 (camera 系) ──
  bool        solved = false;          ///< PnP 是否成功
  Eigen::Vector3d position_camera;     ///< 靶心在相机系下坐标 [m]
  Eigen::Matrix3d rotation_camera;     ///< 靶面在相机系下的旋转矩阵 (靶面法向)

  // ── 世界系位姿 ──
  Eigen::Vector3d position_world;      ///< 靶心在世界系下坐标 [m]
  Eigen::Vector3d ypr_in_world;        ///< 世界系下欧拉角 (yaw, pitch, roll) [rad]

  // ── 时间戳 ──
  std::chrono::steady_clock::time_point timestamp;

  // ── 图像尺寸 (归一化用) ──
  cv::Size     image_size;

  /**
   * @brief 归一化中心 x (用于串口粗瞄准)
   * @return [-1, 1] 范围，0 表示图像中心
   */
  float center_norm_x() const {
    if (image_size.width <= 0) return 0.f;
    return center.x / static_cast<float>(image_size.width) * 2.f - 1.f;
  }

  /** @brief 靶面法向在相机系下的单位向量 */
  Eigen::Vector3d normal_camera() const {
    return rotation_camera.col(2);  // z-axis of target frame = outward normal
  }

  // ── 4 角点 (左上→右上→右下→左下) ──
  std::vector<cv::Point2f> corners() const {
    return {
      cv::Point2f(static_cast<float>(bbox.x), static_cast<float>(bbox.y)),
      cv::Point2f(static_cast<float>(bbox.x + bbox.width), static_cast<float>(bbox.y)),
      cv::Point2f(static_cast<float>(bbox.x + bbox.width), static_cast<float>(bbox.y + bbox.height)),
      cv::Point2f(static_cast<float>(bbox.x), static_cast<float>(bbox.y + bbox.height))
    };
  }
};

/**
 * @struct DartDetectionResult
 * @brief 单帧检测结果 — 包含所有检测到的靶
 */
struct DartDetectionResult
{
  std::vector<DartTarget> targets;
  std::chrono::steady_clock::time_point timestamp;
  int frame_id = 0;
};

}  // namespace dart_auto_aim

#endif  // DART_AUTO_AIM__DART_TYPES_HPP
