cmake_minimum_required(VERSION 3.8)
project(rviz_trajectory_player)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdfdom REQUIRED)

add_executable(trajectory_republisher src/trajectory_republisher.cpp)
ament_target_dependencies(trajectory_republisher rclcpp sensor_msgs urdfdom)

target_include_directories(trajectory_republisher SYSTEM PUBLIC ${urdfdom_INCLUDE_DIRS})

install(DIRECTORY
  launch
  config
  worlds_and_models
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS trajectory_republisher DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
