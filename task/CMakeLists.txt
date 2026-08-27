# test
add_executable(read_toml test/read_toml.cpp)
add_executable(test_serial test/test_serial.cpp)
add_executable(test_logger test/test_logger.cpp)
add_executable(test_foxglove test/test_foxglove.cpp)
add_executable(test_camera test/test_camera.cpp)
add_executable(test_mid360 test/test_mid360.cpp)
add_executable(test_point_lio test/test_point_lio.cpp)
add_executable(test_auto_aim test/test_auto_aim.cpp)

target_link_libraries(test_serial PRIVATE io tools)
target_link_libraries(test_logger PRIVATE tools)
target_link_libraries(test_foxglove PRIVATE tools ${OpenCV_LIBS})
target_link_libraries(test_camera PRIVATE tools ${OpenCV_LIBS} io)
target_link_libraries(test_mid360 PRIVATE tools io)
target_link_libraries(test_point_lio PRIVATE tools io point_lio)
target_link_libraries(test_auto_aim PRIVATE app_auto_aim ${OpenCV_LIBS})

# task
add_executable(sentry sentry.cpp)

target_link_libraries(sentry PRIVATE tools io app_auto_aim ${OpenCV_LIBS})
