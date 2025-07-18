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

# === Build C++ Executables ===
add_executable(go_home_node src/go_home.cpp)
ament_target_dependencies(go_home_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

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
  DESTINATION lib/${PROJECT_NAME}
)

# === Install Python Executables ===
install(PROGRAMS
  scripts/trajectory_logger.py
  scripts/plot_trajectory_log.py
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

# === Linting / Testing (optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === Finalize ===
ament_package()