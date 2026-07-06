/**
 * @file dart_solver.h
 * @brief 角度解算 — 像素中心 + 镗准线 → yaw/pitch 角度误差（度）
 *
 * 移植自 master2 vision/dart_solver。用相机内参 + 畸变系数做 undistortPoints，
 * 再 atan 求相对镗准线的角度误差。零框架依赖。
 */
#ifndef DART_SOLVER_H
#define DART_SOLVER_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "dart_detector.h"

class DartSolver {
public:
    DartSolver();

    /** 设置相机内参（3x3）与畸变系数（1xN） */
    void set_camera(const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs);

    /** 从 yaml 加载相机内参（camera_matrix + distortion_coefficients） */
    bool load_camera(const std::string &path);

    /** 设置镗准线像素坐标（默认画面中心） */
    void set_boresight(const cv::Point2f &bs);

    /** 设置目标平面位姿（object/plane -> camera，单位随 tvec/object points，推荐 mm） */
    bool set_plane_pose(const cv::Mat &rvec, const cv::Mat &tvec);

    /** 由目标平面 3D 点与图像点 solvePnP 求外参 */
    bool set_plane_pose_from_points(const std::vector<cv::Point3f> &object_points_mm,
                                    const std::vector<cv::Point2f> &image_points);

    /** 从 YAML 读取 object_points_mm/image_points 并求目标平面外参 */
    bool load_plane_pose(const std::string &path);

    /** 目标平面外参是否可用 */
    bool plane_ready() const { return plane_pose_ready_; }

    /** 解算 target 的 yaw_err/pitch_err（度）和平面偏差（mm），无目标时清零 */
    void solve(DartTarget &target);

    /** 镗准线像素（可视化用） */
    cv::Point2f boresight() const { return boresight_; }

private:
    cv::Mat     camera_matrix_;
    cv::Mat     distort_coeffs_;
    cv::Point2f boresight_;

    cv::Mat rvec_;                 // plane/world -> camera
    cv::Mat tvec_;                 // plane/world -> camera，单位 mm
    cv::Mat R_cw_;                 // plane/world -> camera
    cv::Mat R_wc_;                 // camera -> plane/world
    cv::Mat camera_center_w_;      // 相机中心在目标平面坐标系下
    bool    plane_pose_ready_ = false;

    bool intersect_plane_z0(const cv::Point2f &pixel, cv::Point2f &plane_point_mm) const;
    static bool read_point2f_sequence(const cv::FileNode &node, std::vector<cv::Point2f> &points);
    static bool read_point3f_sequence(const cv::FileNode &node, std::vector<cv::Point3f> &points);
};

#endif
