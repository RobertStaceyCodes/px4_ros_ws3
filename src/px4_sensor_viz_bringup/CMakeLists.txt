cmake_minimum_required(VERSION 3.8)
project(px4_sensor_viz_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/tf_sanitizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES README.md OAK_D_LITE_SETUP.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
