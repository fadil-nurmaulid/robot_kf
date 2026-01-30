cmake_minimum_required(VERSION 3.8)
project(robot_kf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

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

add_executable(noisy_odom_node src/noisy_odom_node.cpp)

ament_target_dependencies(noisy_odom_node 
  rclcpp 
  nav_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(kalman_odom_node src/kalman_odom_node.cpp)

ament_target_dependencies(kalman_odom_node 
  rclcpp 
  nav_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  tf2_ros
)

target_link_libraries(kalman_odom_node 
  Eigen3::Eigen
)

install(TARGETS
  noisy_odom_node
  kalman_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
