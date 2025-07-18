cmake_minimum_required(VERSION 3.8)
project(robot_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs)
find_package(sensor_msgs REQUIRED)

add_executable(square src/square.cpp)
ament_target_dependencies(square rclcpp geometry_msgs)

add_executable(square_odom src/square_odom.cpp)
ament_target_dependencies(square_odom rclcpp nav_msgs)

add_executable(wandering src/wandering.cpp)
ament_target_dependencies(wandering rclcpp geometry_msgs nav_msgs sensor_msgs)

install(TARGETS 
  square
  square_odom
  wandering
  DESTINATION lib/${PROJECT_NAME}
)

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
