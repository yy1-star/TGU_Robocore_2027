cmake_minimum_required(VERSION 3.16)

add_library(app_auto_aim STATIC
    auto_aim/auto_aim.cpp
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

add_subdirectory(point_lio)
