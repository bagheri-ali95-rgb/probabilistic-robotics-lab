cmake_minimum_required(VERSION 3.8)
project(prob_rob_lab_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(talker src/pub.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/sub.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(cmd_vel_pub src/cmd_vel_pub.cpp)
ament_target_dependencies(cmd_vel_pub rclcpp geometry_msgs)

add_executable(kf_node src/kf_node.cpp)
ament_target_dependencies(kf_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(pf_node src/pf_node.cpp)
ament_target_dependencies(pf_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  talker listener cmd_vel_pub kf_node ekf_node pf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
