cmake_minimum_required(VERSION 3.8)
project(spraying_pathways)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # For Python support
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_py REQUIRED)
find_package(moveit_core REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# === Build C++ Executables ===
add_executable(go_home_node src/go_home.cpp)
ament_target_dependencies(go_home_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  yaml-cpp
)

target_link_libraries(go_home_node yaml-cpp)

add_executable(follow_joints_positions_node src/follow_joints_positions.cpp)
ament_target_dependencies(follow_joints_positions_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_v2_node src/cartesian_path_planner_cubes_v2.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_v2_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_trajectory_v1_node src/cartesian_path_planner_trajectory_v1.cpp)
ament_target_dependencies(cartesian_path_planner_trajectory_v1_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_v3_node src/cartesian_path_planner_cubes_v3.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_v3_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_test_v1_node src/cartesian_path_planner_cubes_test_v1.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_test_v1_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_test_v2_node src/cartesian_path_planner_cubes_test_v2.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_test_v2_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_test_go_v1_node src/cartesian_path_planner_cubes_test_go_v1.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_test_go_v1_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_test_go_v2_node src/cartesian_path_planner_cubes_test_go_v2.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_test_go_v2_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(cartesian_path_planner_cubes_v4_node src/cartesian_path_planner_cubes_v4.cpp)
ament_target_dependencies(cartesian_path_planner_cubes_v4_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

add_executable(lidar_surface_scanner_node src/lidar_surface_scanner.cpp)
ament_target_dependencies(lidar_surface_scanner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
)

add_executable(flat_fan_spraying_node src/flat_fan_spraying.cpp)
ament_target_dependencies(flat_fan_spraying_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  visualization_msgs
)

add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  visualization_msgs
  moveit_core
  moveit_ros_planning
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(test_node yaml-cpp)

add_executable(flat_fan_spraying_v2_node src/flat_fan_spraying_v2.cpp)
ament_target_dependencies(flat_fan_spraying_v2_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  visualization_msgs
  moveit_core
  moveit_ros_planning
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(flat_fan_spraying_v2_node yaml-cpp)

add_executable(moving_obstacle src/moving_obstacle.cpp)
ament_target_dependencies(moving_obstacle
  rclcpp
  geometry_msgs
  gazebo_msgs
  ament_index_cpp
)


# === Install C++ Executables ===
install(TARGETS
  go_home_node
  follow_joints_positions_node
  cartesian_path_planner_cubes_v2_node
  cartesian_path_planner_trajectory_v1_node
  cartesian_path_planner_cubes_v3_node
  cartesian_path_planner_cubes_test_v1_node
  cartesian_path_planner_cubes_test_v2_node
  cartesian_path_planner_cubes_test_go_v1_node
  cartesian_path_planner_cubes_test_go_v2_node
  cartesian_path_planner_cubes_v4_node
  lidar_surface_scanner_node
  flat_fan_spraying_node
  test_node
  flat_fan_spraying_v2_node
  moving_obstacle
  DESTINATION lib/${PROJECT_NAME}
)

# === Install Python Executables ===
install(PROGRAMS
  scripts/trajectory_logger.py
  scripts/plot_trajectory_log.py
  scripts/depth_dip_detector_v1.py
  scripts/transform_camera_pointcloud.py
  scripts/ee_velocity_monitor.py
  scripts/unknown_object_detector.py
  scripts/pointcloud_transform_and_unknown_filter.py
  scripts/pointcloud_transform_and_unknown_filter_v2.py
  DESTINATION lib/${PROJECT_NAME}
)

# === Install Other Resources ===
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)


# === Linting / Testing (optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === Finalize ===
ament_package()