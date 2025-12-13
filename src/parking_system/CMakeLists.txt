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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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
  scripts/select_slot.py
  scripts/test_parking_slot_selection.py
  scripts/temporary_odom_publisher.py
  scripts/slam_initializer.py
  scripts/map_bootstrap.py
  scripts/odom_bootstrap.py
  scripts/localization_initializer.py
  scripts/diagnose_scan_quality.py
  scripts/diagnose_localization.py
  scripts/diagnose_tf_tree.py
  scripts/print_robot_position.py
  scripts/parking_system/parking_coordinator.py
  scripts/parking_system/parking_slot_selector.py
  scripts/parking_system/path_monitor.py
  scripts/parking_system/path_analyzer.py
  scripts/parking_system/aruco_detector.py
  scripts/parking_system/sensor_fusion.py
  scripts/parking_system/aruco_navigation.py
  scripts/parking_system/marker_selector.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

