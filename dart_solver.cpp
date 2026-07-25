/**
 * @file dart_solver.cpp
 * @brief 角度解算 + 目标平面射线求交实现
 */
#include "dart_solver.h"

#include <cmath>
#include <iostream>
#include <vector>

DartSolver::DartSolver() {
    // 占位内参（CLAUDE.md 中的 FX/FY/CX/CY），后续标定替换
    double cm[9] = {800, 0, 320,
                    0, 800, 240,
                    0,   0,   1};
    camera_matrix_ = cv::Mat(3, 3, CV_64F, cm).clone();
    distort_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    boresight_ = cv::Point2f(320.f, 240.f);   // 画面中心
}

void DartSolver::set_camera(const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs) {
    camera_matrix_  = camera_matrix.clone();
    distort_coeffs_ = distort_coeffs.clone();
}

bool DartSolver::load_camera(const std::string &path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[Solver] camera intrinsics config not found: " << path << std::endl;
        return false;
    }
    cv::Mat K, D;
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;
    if (K.empty() || K.rows != 3 || K.cols != 3) {
        std::cerr << "[Solver] invalid camera_matrix in: " << path << std::endl;
        return false;
    }
    K.convertTo(camera_matrix_, CV_64F);
    if (!D.empty()) D.convertTo(distort_coeffs_, CV_64F);
    std::cout << "[Solver] camera intrinsics loaded: " << path
              << " fx=" << camera_matrix_.at<double>(0,0)
              << " fy=" << camera_matrix_.at<double>(1,1)
              << " cx=" << camera_matrix_.at<double>(0,2)
              << " cy=" << camera_matrix_.at<double>(1,2) << std::endl;
    return true;
}

void DartSolver::set_boresight(const cv::Point2f &bs) { boresight_ = bs; }

bool DartSolver::load_boresight(const std::string &path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    float x = boresight_.x, y = boresight_.y;
    if (!fs["boresight_x"].empty()) x = float((double)fs["boresight_x"]);
    if (!fs["boresight_y"].empty()) y = float((double)fs["boresight_y"]);
    boresight_ = cv::Point2f(x, y);
    std::cout << "[Solver] boresight loaded: (" << x << "," << y << ")" << std::endl;
    return true;
}

bool DartSolver::save_boresight(const std::string &path) const {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "boresight_x" << boresight_.x;
    fs << "boresight_y" << boresight_.y;
    return true;
}

bool DartSolver::set_plane_pose(const cv::Mat &rvec, const cv::Mat &tvec) {
    if (rvec.empty() || tvec.empty()) return false;

    rvec_ = rvec.clone();
    tvec_ = tvec.clone();
    rvec_ = rvec_.reshape(1, 3);
    tvec_ = tvec_.reshape(1, 3);
    rvec_.convertTo(rvec_, CV_64F);
    tvec_.convertTo(tvec_, CV_64F);

    cv::Rodrigues(rvec_, R_cw_);
    R_wc_ = R_cw_.t();
    camera_center_w_ = -R_wc_ * tvec_;
    plane_pose_ready_ = true;
    return true;
}

bool DartSolver::set_plane_pose_from_points(const std::vector<cv::Point3f> &object_points_mm,
                                             const std::vector<cv::Point2f> &image_points) {
    if (object_points_mm.size() < 4 || object_points_mm.size() != image_points.size()) return false;

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(object_points_mm, image_points,
                           camera_matrix_, distort_coeffs_,
                           rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!ok) return false;
    return set_plane_pose(rvec, tvec);
}

bool DartSolver::read_point2f_sequence(const cv::FileNode &node, std::vector<cv::Point2f> &points) {
    if (!node.isSeq()) return false;
    points.clear();
    for (const auto &item : node) {
        if (!item.isSeq() || item.size() < 2) return false;
        cv::FileNodeIterator it = item.begin();
        float x = float(*it); ++it;
        float y = float(*it);
        points.emplace_back(x, y);
    }
    return true;
}

bool DartSolver::read_point3f_sequence(const cv::FileNode &node, std::vector<cv::Point3f> &points) {
    if (!node.isSeq()) return false;
    points.clear();
    for (const auto &item : node) {
        if (!item.isSeq() || item.size() < 3) return false;
        cv::FileNodeIterator it = item.begin();
        float x = float(*it); ++it;
        float y = float(*it); ++it;
        float z = float(*it);
        points.emplace_back(x, y, z);
    }
    return true;
}

bool DartSolver::load_plane_pose(const std::string &path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[Solver] plane pose config not found: " << path << std::endl;
        plane_pose_ready_ = false;
        return false;
    }

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    if (!read_point3f_sequence(fs["object_points_mm"], object_points) ||
        !read_point2f_sequence(fs["image_points"], image_points)) {
        std::cerr << "[Solver] invalid plane pose config: " << path << std::endl;
        plane_pose_ready_ = false;
        return false;
    }

    if (!set_plane_pose_from_points(object_points, image_points)) {
        std::cerr << "[Solver] solvePnP failed for: " << path << std::endl;
        plane_pose_ready_ = false;
        return false;
    }

    std::cout << "[Solver] plane pose loaded: " << path
              << " points=" << object_points.size() << std::endl;
    return true;
}

bool DartSolver::intersect_plane_z0(const cv::Point2f &pixel, cv::Point2f &plane_point_mm) const {
    if (!plane_pose_ready_) return false;

    std::vector<cv::Point2f> src{pixel};
    std::vector<cv::Point2f> norm;
    cv::undistortPoints(src, norm, camera_matrix_, distort_coeffs_);
    if (norm.empty()) return false;

    cv::Mat dir_c = (cv::Mat_<double>(3, 1) << double(norm[0].x), double(norm[0].y), 1.0);
    cv::Mat dir_w = R_wc_ * dir_c;

    double cz = camera_center_w_.at<double>(2, 0);
    double dz = dir_w.at<double>(2, 0);
    if (std::abs(dz) < 1e-9) return false;

    double s = -cz / dz;
    if (!std::isfinite(s) || s <= 0.0) return false;

    cv::Mat p = camera_center_w_ + s * dir_w;
    double x = p.at<double>(0, 0);
    double y = p.at<double>(1, 0);
    if (!std::isfinite(x) || !std::isfinite(y)) return false;

    plane_point_mm = cv::Point2f(float(x), float(y));
    return true;
}

void DartSolver::solve(DartTarget &target) {
    target.plane_point_mm = cv::Point2f(0.f, 0.f);
    target.plane_offset_mm = cv::Point2f(0.f, 0.f);
    target.cam_x_mm     = 0.f;
    target.cam_depth_mm = 0.f;
    target.plane_valid = false;

    if (!target.found) {
        target.yaw_err   = 0.f;
        target.pitch_err = 0.f;
        return;
    }

    std::vector<cv::Point2f> src{target.center, boresight_};
    std::vector<cv::Point2f> norm;
    cv::undistortPoints(src, norm, camera_matrix_, distort_coeffs_);

    double yaw   = std::atan(norm[0].x) - std::atan(norm[1].x);
    double pitch = std::atan(norm[1].y) - std::atan(norm[0].y);

    target.yaw_err   = float(yaw   * 180.0 / CV_PI);
    target.pitch_err = float(pitch * 180.0 / CV_PI);

    cv::Point2f target_plane, boresight_plane;
    if (intersect_plane_z0(target.center, target_plane) &&
        intersect_plane_z0(boresight_, boresight_plane)) {
        target.plane_point_mm = target_plane;
        target.plane_offset_mm = cv::Point2f(target_plane.x - boresight_plane.x,
                                             target_plane.y - boresight_plane.y);

        // 把靶面交点转回相机坐标系：P_cam = R_cw * [x,y,0] + tvec
        // 相机系 X=水平(右正)，Z=前向深度；原点=相机/发射点。
        cv::Mat pw = (cv::Mat_<double>(3, 1) << double(target_plane.x),
                                                double(target_plane.y), 0.0);
        cv::Mat pc = R_cw_ * pw + tvec_;
        target.cam_x_mm     = float(pc.at<double>(0, 0));   // 水平偏移
        target.cam_depth_mm = float(pc.at<double>(2, 0));   // 前向深度
        target.plane_valid = true;
    }
}
