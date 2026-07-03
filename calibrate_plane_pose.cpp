#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

struct CaptureSpec {
    std::string image;
    cv::Point2f center_mm{0.f, 0.f};
    float yaw_deg = 0.f;
};

static void usage(const char *argv0) {
    std::cerr << "Usage:\n"
              << "  Single image:\n"
              << "    " << argv0 << " <image> <corners_x> <corners_y> <square_mm> [output.yml]\n"
              << "  Multi-position plane calibration:\n"
              << "    " << argv0 << " --multi <captures.yml> [output.yml]\n\n"
              << "Notes:\n"
              << "  corners_x/corners_y are inner chessboard corner counts.\n"
              << "  Plane coordinate system: X right, Y up, Z=0, unit mm.\n";
}

static bool read_point2(const cv::FileNode &node, cv::Point2f &point) {
    if (!node.isSeq() || node.size() < 2) return false;
    cv::FileNodeIterator it = node.begin();
    point.x = float(*it); ++it;
    point.y = float(*it);
    return true;
}

static std::vector<cv::Point3f> make_board_points(int corners_x, int corners_y, float square_mm,
                                                   const cv::Point2f &center_mm, float yaw_deg) {
    std::vector<cv::Point3f> points;
    points.reserve(size_t(corners_x * corners_y));

    const float origin_x = (corners_x - 1) * square_mm * 0.5f;
    const float origin_y = (corners_y - 1) * square_mm * 0.5f;
    const float yaw = yaw_deg * float(CV_PI) / 180.f;
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);

    for (int y = 0; y < corners_y; ++y) {
        for (int x = 0; x < corners_x; ++x) {
            const float bx = x * square_mm - origin_x;
            const float by = origin_y - y * square_mm;
            const float tx = center_mm.x + c * bx - s * by;
            const float ty = center_mm.y + s * bx + c * by;
            points.emplace_back(tx, ty, 0.f);
        }
    }
    return points;
}

static bool detect_chessboard(const std::string &image_path, int corners_x, int corners_y,
                              std::vector<cv::Point2f> &image_points, cv::Mat *preview = nullptr) {
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Failed to read image: " << image_path << std::endl;
        return false;
    }

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    const cv::Size pattern_size(corners_x, corners_y);
    bool found = cv::findChessboardCorners(gray, pattern_size, image_points,
                                           cv::CALIB_CB_ADAPTIVE_THRESH |
                                           cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!found) {
        std::cerr << "Chessboard not found in: " << image_path << std::endl;
        return false;
    }

    cv::cornerSubPix(gray, image_points, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));

    if (preview) {
        image.copyTo(*preview);
        cv::drawChessboardCorners(*preview, pattern_size, image_points, found);
    }
    return true;
}

static bool write_pose_yaml(const std::string &output_path,
                            const std::vector<cv::Point3f> &object_points,
                            const std::vector<cv::Point2f> &image_points) {
    if (object_points.empty() || object_points.size() != image_points.size()) return false;

    cv::FileStorage fs(output_path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open output: " << output_path << std::endl;
        return false;
    }

    fs << "object_points_mm" << "[";
    for (const auto &p : object_points) {
        fs << "[" << p.x << p.y << p.z << "]";
    }
    fs << "]";

    fs << "image_points" << "[";
    for (const auto &p : image_points) {
        fs << "[" << p.x << p.y << "]";
    }
    fs << "]";
    return true;
}

static int run_single(int argc, char **argv) {
    const std::string image_path = argv[1];
    const int corners_x = std::atoi(argv[2]);
    const int corners_y = std::atoi(argv[3]);
    const float square_mm = std::atof(argv[4]);
    const std::string output_path = (argc >= 6) ? argv[5] : "config/dart_plane_pose.yml";

    if (corners_x <= 1 || corners_y <= 1 || square_mm <= 0.f) {
        usage(argv[0]);
        return 1;
    }

    std::vector<cv::Point2f> image_points;
    cv::Mat preview;
    if (!detect_chessboard(image_path, corners_x, corners_y, image_points, &preview)) return 1;

    std::vector<cv::Point3f> object_points =
        make_board_points(corners_x, corners_y, square_mm, cv::Point2f(0.f, 0.f), 0.f);

    if (!write_pose_yaml(output_path, object_points, image_points)) return 1;

    const std::string preview_path = output_path + ".preview.jpg";
    cv::imwrite(preview_path, preview);

    std::cout << "Wrote " << output_path << " with " << image_points.size() << " points\n"
              << "Preview: " << preview_path << std::endl;
    return 0;
}

static int run_multi(int argc, char **argv) {
    if (argc < 3 || argc > 4) {
        usage(argv[0]);
        return 1;
    }

    const std::string config_path = argv[2];
    const std::string output_path = (argc >= 4) ? argv[3] : "config/dart_plane_pose.yml";

    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open multi config: " << config_path << std::endl;
        return 1;
    }

    const int corners_x = int(fs["corners_x"]);
    const int corners_y = int(fs["corners_y"]);
    const float square_mm = float(fs["square_mm"]);
    if (corners_x <= 1 || corners_y <= 1 || square_mm <= 0.f || !fs["captures"].isSeq()) {
        std::cerr << "Invalid multi config: " << config_path << std::endl;
        return 1;
    }

    std::vector<CaptureSpec> captures;
    for (const auto &node : fs["captures"]) {
        CaptureSpec cap;
        node["image"] >> cap.image;
        if (cap.image.empty() || !read_point2(node["board_center_mm"], cap.center_mm)) {
            std::cerr << "Invalid capture entry in: " << config_path << std::endl;
            return 1;
        }
        cap.yaw_deg = node["board_yaw_deg"].empty() ? 0.f : float(node["board_yaw_deg"]);
        captures.push_back(cap);
    }

    if (captures.empty()) {
        std::cerr << "No captures in: " << config_path << std::endl;
        return 1;
    }

    std::vector<cv::Point3f> all_object_points;
    std::vector<cv::Point2f> all_image_points;
    const int points_per_image = corners_x * corners_y;
    all_object_points.reserve(size_t(points_per_image * captures.size()));
    all_image_points.reserve(size_t(points_per_image * captures.size()));

    int ok_count = 0;
    for (const auto &cap : captures) {
        std::vector<cv::Point2f> image_points;
        cv::Mat preview;
        if (!detect_chessboard(cap.image, corners_x, corners_y, image_points, &preview)) return 1;

        std::vector<cv::Point3f> object_points =
            make_board_points(corners_x, corners_y, square_mm, cap.center_mm, cap.yaw_deg);

        all_object_points.insert(all_object_points.end(), object_points.begin(), object_points.end());
        all_image_points.insert(all_image_points.end(), image_points.begin(), image_points.end());

        const std::string preview_path = cap.image + ".preview.jpg";
        cv::imwrite(preview_path, preview);
        std::cout << "Capture OK: " << cap.image
                  << " center=(" << cap.center_mm.x << "," << cap.center_mm.y << ")"
                  << " yaw=" << cap.yaw_deg
                  << " preview=" << preview_path << std::endl;
        ok_count++;
    }

    if (!write_pose_yaml(output_path, all_object_points, all_image_points)) return 1;

    std::cout << "Wrote " << output_path
              << " captures=" << ok_count
              << " total_points=" << all_image_points.size() << std::endl;
    return 0;
}

int main(int argc, char **argv) {
    if (argc >= 2 && std::string(argv[1]) == "--multi") {
        return run_multi(argc, argv);
    }

    if (argc < 5 || argc > 6) {
        usage(argv[0]);
        return 1;
    }
    return run_single(argc, argv);
}
