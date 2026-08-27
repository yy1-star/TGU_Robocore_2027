cmake_minimum_required(VERSION 3.16)

add_library(app_auto_aim STATIC
    auto_aim/aimer.cpp
    auto_aim/auto_aim.cpp
    auto_aim/armor.cpp
    auto_aim/classifier.cpp
    auto_aim/detector.cpp
    auto_aim/solver.cpp
    auto_aim/target.cpp
    auto_aim/tracker.cpp
)

target_include_directories(app_auto_aim
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(app_auto_aim
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        tools
)

add_subdirectory(point_lio EXCLUDE_FROM_ALL)
