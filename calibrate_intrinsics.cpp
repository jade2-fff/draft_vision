// 相机内参标定工具：多张棋盘格图 → 算 fx/fy/cx/cy + 畸变，输出 yaml
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

static void usage(const char *a0) {
    std::fprintf(stderr,
        "Usage: %s <corners_x> <corners_y> <square_mm> <out.yml> <img1> [img2 ...]\n"
        "  corners_x/corners_y: 棋盘格内角点数（不是格子数）\n"
        "  square_mm: 每格真实边长 mm\n"
        "  建议 15~20 张，标定板在相机前不同角度/位置/远近，布满画面各区域\n", a0);
}

int main(int argc, char **argv) {
    if (argc < 6) { usage(argv[0]); return 1; }

    const int   cx = std::atoi(argv[1]);
    const int   cy = std::atoi(argv[2]);
    const float sq = std::atof(argv[3]);
    const std::string out = argv[4];
    if (cx <= 1 || cy <= 1 || sq <= 0.f) { usage(argv[0]); return 1; }

    const cv::Size pattern(cx, cy);

    // 棋盘格在自身坐标系的 3D 点（Z=0）
    std::vector<cv::Point3f> objp;
    for (int y = 0; y < cy; ++y)
        for (int x = 0; x < cx; ++x)
            objp.emplace_back(x * sq, y * sq, 0.f);

    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;
    cv::Size img_size;
    int ok = 0, fail = 0;

    for (int i = 5; i < argc; ++i) {
        cv::Mat im = cv::imread(argv[i], cv::IMREAD_COLOR);
        if (im.empty()) { std::fprintf(stderr, "读图失败: %s\n", argv[i]); ++fail; continue; }
        img_size = im.size();

        cv::Mat gray;
        cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, pattern, corners,
                        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!found) { std::fprintf(stderr, "未检测到棋盘格: %s\n", argv[i]); ++fail; continue; }

        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));

        obj_points.push_back(objp);
        img_points.push_back(corners);
        ++ok;
        std::printf("OK: %s\n", argv[i]);
    }

    if (ok < 3) {
        std::fprintf(stderr, "有效图太少(%d)，至少需要 3 张，建议 15+\n", ok);
        return 1;
    }

    cv::Mat K, D;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(obj_points, img_points, img_size, K, D, rvecs, tvecs);

    std::printf("\n标定完成：有效 %d 张，失败 %d 张\n", ok, fail);
    std::printf("重投影误差 RMS = %.4f 像素（<0.5 很好，<1.0 可用，>1.5 需重拍）\n", rms);
    std::printf("图像尺寸 = %dx%d\n", img_size.width, img_size.height);
    std::printf("fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n",
        K.at<double>(0,0), K.at<double>(1,1), K.at<double>(0,2), K.at<double>(1,2));

    cv::FileStorage fs(out, cv::FileStorage::WRITE);
    if (!fs.isOpened()) { std::fprintf(stderr, "无法写入: %s\n", out.c_str()); return 1; }
    fs << "image_width"  << img_size.width;
    fs << "image_height" << img_size.height;
    fs << "camera_matrix" << K;
    fs << "distortion_coefficients" << D;
    fs << "rms_reproj_error" << rms;
    fs.release();

    std::printf("已写入: %s\n", out.c_str());
    return 0;
}
