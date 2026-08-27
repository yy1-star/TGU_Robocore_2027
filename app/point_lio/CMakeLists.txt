cmake_minimum_required(VERSION 3.16)

add_library(point_lio STATIC
    point_lio.cpp
    point_lio_config.cpp
)

target_include_directories(point_lio
    PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(point_lio
    PUBLIC
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        tools
        io
)
