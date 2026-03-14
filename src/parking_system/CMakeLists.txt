cmake_minimum_required(VERSION 3.8)
project(parking_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install Python scripts
ament_python_install_package(parking_system
  PACKAGE_DIR scripts)

# Install launch files
install(DIRECTORY
  launch
  config
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts (standalone scripts)
install(PROGRAMS
  scripts/diagnose_scan_quality.py
  scripts/diagnose_localization.py
  scripts/diagnose_tf_tree.py
  scripts/diagnose_control_chain.py
  scripts/record_control_chain_bag.py
  scripts/print_robot_position.py
  scripts/nav2_activator.py
  scripts/cmd_vel_to_pico_bridge.py
  scripts/ros2_mobile_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
