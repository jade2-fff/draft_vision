#include "tasks/dart_auto_aim/solver/dart_solver.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace dart_auto_aim
{

DartSolver::DartSolver(const std::string & config_path)
{
  YAML::Node config = YAML::LoadFile(config_path);

  // ── 相机内参 ──
  auto cm = config["camera_matrix"].as<std::vector<double>>();
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  std::memcpy(camera_matrix_.data, cm.data(), 9 * sizeof(double));

  auto dc = config["distort_coeffs"].as<std::vector<double>>();
  distort_coeffs_ = cv::Mat(1, static_cast<int>(dc.size()), CV_64F);
  std::memcpy(distort_coeffs_.data, dc.data(), dc.size() * sizeof(double));

  // ── 手眼标定 ──
  auto rvec = config["R_camera2gimbal"].as<std::vector<double>>();
  if (rvec.size() == 9) {
    R_camera2gimbal_ = Eigen::Map<Eigen::Matrix3d>(rvec.data());
  } else {
    R_camera2gimbal_ = Eigen::Matrix3d::Identity();
  }

  auto tvec = config["t_camera2gimbal"].as<std::vector<double>>();
  if (tvec.size() == 3) {
    t_camera2gimbal_ = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);
  } else {
    t_camera2gimbal_ = Eigen::Vector3d::Zero();
  }

  // ── 靶面尺寸 ──
  if (config["dart_target"]) {
    target_half_size_ = config["dart_target"]["half_size"].as<double>(target_half_size_);
  }

  R_gimbal2world_ = Eigen::Matrix3d::Identity();

  tools::logger()->info("[DartSolver] Init: K=[{:.1f},{:.1f}] half={:.3f}m",
    camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1), target_half_size_);
}

void DartSolver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  R_gimbal2world_ = q.toRotationMatrix();
}

bool DartSolver::solve(DartTarget & target) const
{
  // ── 1. 构建世界点 (靶面四角，z=0 平面即靶面) ──
  float hs = static_cast<float>(target_half_size_);
  std::vector<cv::Point3f> object_points = {
    {-hs, -hs, 0.f},  // 左上
    { hs, -hs, 0.f},  // 右上
    { hs,  hs, 0.f},  // 右下
    {-hs,  hs, 0.f}   // 左下
  };

  // ── 2. 图像角点 (bbox 四角) ──
  std::vector<cv::Point2f> image_points = target.corners();

  // ── 3. PnP 解算 ──
  cv::Mat rvec, tvec;
  bool ok = cv::solvePnP(
    object_points, image_points,
    camera_matrix_, distort_coeffs_,
    rvec, tvec,
    false, cv::SOLVEPNP_ITERATIVE);

  if (!ok) {
    target.solved = false;
    return false;
  }

  // ── 4. 提取结果 ──
  target.position_camera = Eigen::Vector3d(
    tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

  cv::Mat rot_mat;
  cv::Rodrigues(rvec, rot_mat);
  cv::cv2eigen(rot_mat, target.rotation_camera);

  // ── 5. 相机系 → 云台系 ──
  Eigen::Vector3d pos_gimbal = R_camera2gimbal_ * target.position_camera + t_camera2gimbal_;

  // ── 6. 云台系 → 世界系 ──
  Eigen::Vector3d pos_world = R_gimbal2world_ * pos_gimbal;
  target.position_world = pos_world;

  // 世界系下靶面法向
  Eigen::Matrix3d R_target2world = R_gimbal2world_ * R_camera2gimbal_ * target.rotation_camera;
  target.ypr_in_world = tools::eulers(R_target2world, 2, 1, 0);

  target.solved = true;
  return true;
}

}  // namespace dart_auto_aim
