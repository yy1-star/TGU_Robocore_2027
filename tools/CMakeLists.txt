cmake_minimum_required(VERSION 3.16)

add_library(tools STATIC
    crc.cpp
    extended_kalman_filter.cpp
    foxglove_comm.cpp
    logger.cpp
    math_tools.cpp
    trajectory.cpp
    tf.cpp
)

target_include_directories(tools
    PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/tools/foxglove/include
)

target_link_libraries(tools
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${PROJECT_SOURCE_DIR}/tools/foxglove/lib/libfoxglove.so
)
