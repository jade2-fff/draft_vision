cmake_minimum_required(VERSION 3.10)
project(dart_standalone)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

add_executable(dart
    dart_standalone.cpp
    serial_protocol.cpp
    dart_detector.cpp
    dart_confirmer.cpp
    dart_solver.cpp
    dart_aimer.cpp
    recorder.cpp
    io/hikrobot/hikrobot.cpp
)

add_executable(calibrate_plane_pose
    calibrate_plane_pose.cpp
)

add_executable(calibrate_intrinsics
    calibrate_intrinsics.cpp
)

# 海康 MVS SDK
target_include_directories(dart PRIVATE io/hikrobot/include)
target_link_directories(dart PRIVATE io/hikrobot/lib/amd64)

find_package(Threads REQUIRED)
target_link_libraries(dart
    ${OpenCV_LIBS}
    Threads::Threads
    MvCameraControl
    PkgConfig::LIBUSB
)

target_link_libraries(calibrate_plane_pose
    ${OpenCV_LIBS}
)

target_link_libraries(calibrate_intrinsics
    ${OpenCV_LIBS}
)
