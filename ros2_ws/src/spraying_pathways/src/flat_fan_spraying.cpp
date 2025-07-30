#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <algorithm>
#include <set>

std::vector<geometry_msgs::msg::Point> transformed_points;

struct Point2D {
  double x, y;
  bool operator<(const Point2D& other) const {
    if (y != other.y) return y > other.y;
    return x < other.x;
  }
};

struct Pose3D {
  Point2D position;
  double z;
};

struct Cube {
  int id;
  Pose3D pose;
  double height;
};

std::vector<Point2D> sort_rectangle_corners(const std::vector<Point2D>& points) {
  if (points.size() != 4) throw std::runtime_error("Exactly 4 points required");
  auto sorted = points;
  std::sort(sorted.begin(), sorted.end());
  std::vector<Point2D> top(sorted.begin(), sorted.begin() + 2);
  std::vector<Point2D> bottom(sorted.begin() + 2, sorted.end());
  std::sort(top.begin(), top.end(), [](const Point2D& a, const Point2D& b) { return a.x < b.x; });
  std::sort(bottom.begin(), bottom.end(), [](const Point2D& a, const Point2D& b) { return a.x < b.x; });
  return {top[0], top[1], bottom[1], bottom[0]};
}

std::string generate_multi_box_sdf(const std::vector<Cube>& cubes, double cube_size_x, double cube_size_y) {
  std::ostringstream sdf;
  sdf << "<?xml version='1.0'?>\n<sdf version='1.7'>\n<model name='multi_cube'>\n  <static>true</static>\n";

  for (const auto& cube : cubes) {
    sdf << "  <link name='cube_" << cube.id << "'>\n";
    sdf << "    <pose>"
        << cube.pose.position.x << " "
        << cube.pose.position.y << " "
        << cube.pose.z << " 0 0 0</pose>\n";
    sdf << "    <collision name='collision'>\n";
    sdf << "      <geometry>\n";
    sdf << "        <box><size>"
        << cube_size_x << " " << cube_size_y << " " << cube.height << "</size></box>\n";
    sdf << "      </geometry>\n    </collision>\n";
    sdf << "    <visual name='visual'>\n";
    sdf << "      <geometry>\n";
    sdf << "        <box><size>"
        << cube_size_x << " " << cube_size_y << " " << cube.height << "</size></box>\n";
    sdf << "      </geometry>\n";
    sdf << "      <material>\n        <script>\n";
    sdf << "          <uri>file:///ros2_ws/src/spraying_pathways/materials/scripts</uri>\n";
    sdf << "          <name>My/Seaweed</name>\n";
    sdf << "        </script>\n";
    sdf << "        <ambient>1 1 1 0.7</ambient>\n";
    sdf << "        <diffuse>1 1 1 0.7</diffuse>\n";
    sdf << "      </material>\n    </visual>\n  </link>\n";
  }

  sdf << "</model>\n</sdf>\n";
  return sdf.str();
}

void generate_grid(const std::vector<Point2D>& area_corners,
                   double spray_width,
                   double spray_length,
                   int N,
                   std::vector<Cube>& cubes,
                   std::vector<Point2D>& spray_centers,
                   double& cube_size_x,
                   double& cube_size_y) {
  double dx = area_corners[1].x - area_corners[0].x;
  double dy = area_corners[3].y - area_corners[0].y;

  double total_width = std::hypot(dx, area_corners[1].y - area_corners[0].y);
  double total_length = std::hypot(area_corners[3].x - area_corners[0].x, dy);

  int cols = std::max(1, static_cast<int>(total_width / spray_width));
  int rows = std::max(1, static_cast<int>(total_length / spray_length));

  double step_x = dx / cols;
  double step_y = dy / rows;

  cube_size_x = step_x;
  cube_size_y = step_y / N;

  cubes.clear();
  spray_centers.clear();
  int cube_id = 0;

  for (int i = 0; i < rows; ++i) {
    std::vector<Point2D> row_centers;
    for (int j = 0; j < cols; ++j) {
      double patch_origin_x = area_corners[0].x + j * step_x;
      double patch_origin_y = area_corners[0].y + i * step_y;

      for (int n = 0; n < N; ++n) {
        double center_x = patch_origin_x + step_x / 2.0;
        double center_y = patch_origin_y + (n + 0.5) * cube_size_y;
        cubes.push_back({cube_id++, {{center_x, center_y}, 0.0}, 0.0});
      }

      int mid_idx = N / 2;
      double waypoint_y = patch_origin_y + (mid_idx + 0.5) * cube_size_y;
      row_centers.push_back({patch_origin_x + step_x / 2.0, waypoint_y});
    }
    if (i % 2 == 1) std::reverse(row_centers.begin(), row_centers.end());
    spray_centers.insert(spray_centers.end(), row_centers.begin(), row_centers.end());
  }
}

void apply_flat_spray(std::vector<Cube>& cubes,
                      const std::vector<Point2D>& spray_centers,
                      double cube_size_x, double cube_size_y,
                      double radius, double standard_h, double min_h, double sigma, double z_base) {
  double epsilon = cube_size_x / 100.0;

  for (const auto& sc : spray_centers) {
    for (auto& cube : cubes) {
      double dx = std::abs(cube.pose.position.x - sc.x);
      double dy = std::abs(cube.pose.position.y - sc.y);

      if (dy <= radius && dx <= epsilon) {
        double h = (sigma == 0)
                   ? standard_h
                   : standard_h - (standard_h - min_h) * std::pow(dy / radius, sigma);
        cube.height += h;
        cube.pose.z = z_base + h / 2.0;
      }
    }
  }
}

std::vector<geometry_msgs::msg::Point> extract_problematic_points(double z_threshold, double tolerance = 0.001) {
  std::vector<geometry_msgs::msg::Point> result;
  for (const auto& p : transformed_points) {
    if (p.z < (z_threshold - tolerance)) {
      result.push_back(p);
    }
  }
  return result;
}

sensor_msgs::msg::PointCloud2 create_pointcloud_from_points(const std::vector<geometry_msgs::msg::Point>& points, const std::string& frame_id) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = rclcpp::Clock().now();
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto& p : points) {
    *iter_x = p.x;
    *iter_y = p.y;
    *iter_z = p.z;
    ++iter_x; ++iter_y; ++iter_z;
  }

  return cloud;
}

std::vector<Cube> get_cubes_with_problematic_points(const std::vector<Cube>& cubes,
                                                    const std::vector<geometry_msgs::msg::Point>& points,
                                                    double cube_size_x,
                                                    double cube_size_y) {
  std::vector<Cube> result;
  std::set<int> seen_ids;

  for (const auto& p : points) {
    for (const auto& cube : cubes) {
      double dx = std::abs(cube.pose.position.x - p.x);
      double dy = std::abs(cube.pose.position.y - p.y);
      if (dx <= cube_size_x / 2 && dy <= cube_size_y / 2) {
        if (seen_ids.insert(cube.id).second) {
          result.push_back(cube);
        }
        break;
      }
    }
  }
  return result;
}

void publish_problematic_markers(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
                                 const std::vector<Cube>& cubes_with_issues,
                                 const std::string& frame_id, double cube_size_x, double cube_size_y) {
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < cubes_with_issues.size(); ++i) {
    const auto& cube = cubes_with_issues[i];

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "problematic_cubes";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cube.pose.position.x;
    marker.pose.position.y = cube.pose.position.y;
    marker.pose.position.z = 0.05 +(cube.height / 2.0);//cube.pose.z + cube.height / 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = cube_size_x * 0.5;
    marker.scale.y = cube_size_y * 0.5;
    marker.scale.z = 0.01;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.9;

    marker_array.markers.push_back(marker);
  }

  publisher->publish(marker_array);
}

void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  transformed_points.clear();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    geometry_msgs::msg::Point p;
    p.x = *iter_x;
    p.y = *iter_y;
    p.z = *iter_z;
    transformed_points.push_back(p);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("spray_sim_node");

  auto sub_pc = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/transformed_points", 10, pointCloudCallback);

  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/problematic_cubes_markers", 10);
  auto problematic_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/problematic_points", 10);

  std::vector<Point2D> unordered = {{0.8, 0.0}, {0.8, 0.4}, {1.2, 0.0}, {1.2, 0.4}};
  std::vector<Point2D> corners = sort_rectangle_corners(unordered);

  const double robot_base_x = 0.25;
  const double robot_base_y = 0.0;

  double z_base = 0.765;
  double spray_width = 0.02;
  double spray_length = 0.1;
  int N = 9;
  double radius = spray_length / 2;
  double standard_h = 0.015;
  double min_h = 0.009;
  double sigma = 4;

  double cube_size_x, cube_size_y;
  std::vector<Cube> cubes;
  std::vector<Point2D> spray_centers;

  generate_grid(corners, spray_width, spray_length, N, cubes, spray_centers, cube_size_x, cube_size_y);
  cube_size_x = std::abs(cube_size_x);
  cube_size_y = std::abs(cube_size_y);

  apply_flat_spray(cubes, spray_centers, cube_size_x, cube_size_y, radius, standard_h, min_h, sigma, z_base);

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(60.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 1.0;
  orientation.z = 0.0;
  orientation.w = 0.0;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  for (const auto& sc : spray_centers) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = sc.x - robot_base_x;
    pose.position.y = sc.y - robot_base_y;
    pose.position.z = 0.2;
    pose.orientation = orientation;
    waypoints.push_back(pose);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  double eef_step = 0.01;
  double jump_thresh = 0.0;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_thresh, trajectory);

  if (fraction > 0.9) {
    RCLCPP_INFO(node->get_logger(), "Planned %.1f%% of path. Executing...", fraction * 100);
    move_group.execute(trajectory);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan complete path (%.1f%%)", fraction * 100);
  }

  std::string sdf_content = generate_multi_box_sdf(cubes, std::abs(cube_size_x), std::abs(cube_size_y));
  std::ofstream out("/tmp/multi_cubes.sdf");
  out << sdf_content;
  out.close();

  std::ostringstream cmd;
  cmd << "ros2 run gazebo_ros spawn_entity.py -file /tmp/multi_cubes.sdf -entity all_cubes";
  std::system(cmd.str().c_str());

  for (auto& cube : cubes) {
    cube.pose.position.x -= robot_base_x;
    cube.pose.position.y -= robot_base_y;
  }

  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    auto problematic = extract_problematic_points(0.05 + standard_h);
    auto cloud_msg = create_pointcloud_from_points(problematic, "base_link");
    problematic_cloud_pub->publish(cloud_msg);

    auto cubes_with_issues = get_cubes_with_problematic_points(cubes, problematic, cube_size_x, cube_size_y);
    publish_problematic_markers(marker_pub, cubes_with_issues, "base_link", cube_size_x, cube_size_y);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
