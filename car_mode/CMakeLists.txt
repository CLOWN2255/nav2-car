cmake_minimum_required(VERSION 3.5)

project(car_mode)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(scan_pub src/scan_pub.cpp)
ament_target_dependencies(scan_pub
  "rclcpp"
  "sensor_msgs"
)


install(TARGETS
  scan_pub
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY
     map/
  DESTINATION share/${PROJECT_NAME}/map
)

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)


install(
  DIRECTORY
    worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY
    meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)


ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()