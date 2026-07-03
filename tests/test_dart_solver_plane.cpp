#include <cmath>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "dart_solver.h"

static bool near(float a, float b, float eps = 1e-2f) {
    return std::fabs(a - b) < eps;
}

int main() {
    DartSolver solver;

    cv::Mat camera = (cv::Mat_<double>(3, 3) <<
        800.0, 0.0, 320.0,
        0.0, 800.0, 240.0,
        0.0, 0.0, 1.0);
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
    solver.set_camera(camera, dist);
    solver.set_boresight({320.f, 240.f});

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1000.0);
    if (!solver.set_plane_pose(rvec, tvec)) {
        std::cerr << "set_plane_pose failed" << std::endl;
        return 1;
    }

    DartTarget target;
    target.found = true;
    target.center = {400.f, 200.f};
    solver.solve(target);

    if (!target.plane_valid) {
        std::cerr << "plane_valid is false" << std::endl;
        return 1;
    }
    if (!near(target.plane_point_mm.x, 100.f) || !near(target.plane_point_mm.y, -50.f)) {
        std::cerr << "unexpected plane point: " << target.plane_point_mm << std::endl;
        return 1;
    }
    if (!near(target.plane_offset_mm.x, 100.f) || !near(target.plane_offset_mm.y, -50.f)) {
        std::cerr << "unexpected plane offset: " << target.plane_offset_mm << std::endl;
        return 1;
    }

    target.found = false;
    solver.solve(target);
    if (target.plane_valid || !near(target.plane_offset_mm.x, 0.f) || !near(target.plane_offset_mm.y, 0.f)) {
        std::cerr << "missing target did not clear plane result" << std::endl;
        return 1;
    }

    return 0;
}
