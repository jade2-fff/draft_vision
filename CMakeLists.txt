cmake_minimum_required(VERSION 3.16)

add_library(dart_auto_aim_detect OBJECT
    detector/dart_detector.cpp
)

add_library(dart_auto_aim_solve OBJECT
    solver/dart_solver.cpp
)

add_library(dart_auto_aim_track OBJECT
    tracker/dart_tracker.cpp
)

add_library(dart_auto_aim_fire OBJECT
    aimer/dart_aimer.cpp
)

target_link_libraries(dart_auto_aim_detect PUBLIC ${OpenCV_LIBS} yaml-cpp::yaml-cpp)
target_link_libraries(dart_auto_aim_solve PUBLIC ${OpenCV_LIBS} yaml-cpp::yaml-cpp)
target_link_libraries(dart_auto_aim_track PUBLIC ${OpenCV_LIBS} yaml-cpp::yaml-cpp)
target_link_libraries(dart_auto_aim_fire PUBLIC ${OpenCV_LIBS} yaml-cpp::yaml-cpp)

add_library(dart_auto_aim STATIC
    $<TARGET_OBJECTS:dart_auto_aim_detect>
    $<TARGET_OBJECTS:dart_auto_aim_solve>
    $<TARGET_OBJECTS:dart_auto_aim_track>
    $<TARGET_OBJECTS:dart_auto_aim_fire>
)

target_link_libraries(dart_auto_aim PUBLIC ${OpenCV_LIBS} yaml-cpp::yaml-cpp fmt_local)
