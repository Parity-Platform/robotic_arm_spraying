#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <set>
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <string>

using std::placeholders::_1;

class DepthDipDetector : public rclcpp::Node
{
public:
  DepthDipDetector()
  : rclcpp::Node("depth_dip_detector")
  {
    // Publisher ready now; subscriber will be created AFTER the center move.
    pub_problematic_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/problematic_points", 10);
  }

  void attach_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi)
  {
    move_group_ = std::move(mgi);
  }

  void move_to_center_and_start()
  {
    if (move_group_) {
      const bool ok = moveToCenter();
      if (!ok && require_center_move_success_) {
        RCLCPP_ERROR(this->get_logger(), "Center move failed and require_center_move_success_=true. Not starting.");
        return;
      }
      if (!ok) {
        RCLCPP_WARN(this->get_logger(), "Center move failed — starting anyway.");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Move group not attached — skipping center move.");
    }

    // Start processing only after (attempted) center move
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/wrist3_cam/points", 10, std::bind(&DepthDipDetector::pointcloudCallback, this, _1));
  }

private:
  // ── EDIT THESE VARIABLES AS YOU LIKE ──────────────────────────────────────────
  // Parallelogram corners (x,y) in base frame
  const std::array<std::pair<double,double>,4> corners_ = {{
    {0.8, 0.4}, {1.2, 0.4}, {1.2, 0.0}, {0.8, 0.0}
  }};

  // Spraying & matching
  double spray_width_    = 0.04;   // grid spacing along X/Y for waypoints
  double match_radius_   = 0.02;   // waypoint proximity check (meters)

  // Robot base offset used earlier in your logic
  double robot_base_x_   = 0.25;
  double robot_base_y_   = 0.0;

  // “Problematic” depth thresholding
  double z_threshold_    = 0.07;
  double tolerance_      = 0.007;

  // Move-to-center settings
  std::string move_group_name_ = "manipulator";
  std::string reference_frame_ = "base_link";
  double center_z_             = 0.60;

  // Your “look-down” quaternion (unit 180° about Y)
  double center_qx_ = 0.0, center_qy_ = 1.0, center_qz_ = 0.0, center_qw_ = 0.0;

  // Motion scaling
  double max_vel_scale_ = 0.30;
  double max_acc_scale_ = 0.30;

  // If true, abort subscribing if the move-to-center fails
  bool require_center_move_success_ = false;
  // ─────────────────────────────────────────────────────────────────────────────

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_problematic_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  static geometry_msgs::msg::Quaternion normalizeQuat(double x, double y, double z, double w)
  {
    geometry_msgs::msg::Quaternion q;
    const double n = std::sqrt(x*x + y*y + z*z + w*w);
    if (n < 1e-9) { q.x = q.y = q.z = 0.0; q.w = 1.0; }
    else { q.x = x/n; q.y = y/n; q.z = z/n; q.w = w/n; }
    return q;
  }

  std::pair<double,double> parallelogramCenterXY() const
  {
    // Center of parallelogram = midpoint of diagonal (c0,c2)
    const double cx = 0.5 * (corners_[0].first  + corners_[2].first);
    const double cy = 0.5 * (corners_[0].second + corners_[2].second);
    return {cx, cy};
  }

  bool moveToCenter()
  {
    auto [cx, cy] = parallelogramCenterXY();

    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = reference_frame_;
    target.pose.position.x = cx;
    target.pose.position.y = cy;
    target.pose.position.z = center_z_;
    target.pose.orientation = normalizeQuat(center_qx_, center_qy_, center_qz_, center_qw_);

    move_group_->setPoseReferenceFrame(reference_frame_);
    move_group_->setMaxVelocityScalingFactor(max_vel_scale_);
    move_group_->setMaxAccelerationScalingFactor(max_acc_scale_);
    move_group_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Planning to center failed.");
      return false;
    }
    auto exec_ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!exec_ok) {
      RCLCPP_ERROR(this->get_logger(), "Execution to center failed.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Reached center (%.3f, %.3f, %.3f) with quat [%.3f, %.3f, %.3f, %.3f].",
                cx, cy, center_z_, center_qx_, center_qy_, center_qz_, center_qw_);
    return true;
  }

  std::vector<std::pair<double,double>> generateWaypoints() const
  {
    const double dx = corners_[1].first  - corners_[0].first;
    const double dy = corners_[3].second - corners_[0].second;

    const double length = std::hypot(dx, corners_[1].second - corners_[0].second);
    const double height = std::hypot(corners_[3].first - corners_[0].first, dy);

    int cols = std::max(1, static_cast<int>(std::floor(length / spray_width_)));
    int rows = std::max(1, static_cast<int>(std::floor(height / spray_width_)));

    const double step_x = dx / static_cast<double>(cols);
    const double step_y = dy / static_cast<double>(rows);

    std::vector<std::pair<double,double>> positions;
    positions.reserve(static_cast<size_t>(rows) * static_cast<size_t>(cols));

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        int col = (i % 2 == 0) ? j : (cols - j - 1);
        double x = corners_[0].first + (static_cast<double>(col) + 0.5) * step_x - robot_base_x_;
        double y = corners_[0].second + (static_cast<double>(i) + 0.5) * step_y - robot_base_y_;
        positions.emplace_back(std::round(x * 1e4) / 1e4, std::round(y * 1e4) / 1e4);
      }
    }
    return positions;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<std::array<float,3>> problematic_points;
    problematic_points.reserve(static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height));

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (static_cast<double>(z) < (z_threshold_ - tolerance_)) {
        problematic_points.push_back({x, y, z});
      }
    }

    const auto waypoints = generateWaypoints();
    std::set<std::pair<double,double>> matched_waypoints;

    for (const auto &p : problematic_points) {
      const double px = p[0], py = p[1];
      for (const auto &wp : waypoints) {
        if (std::hypot(px - wp.first, py - wp.second) <= match_radius_) {
          matched_waypoints.insert(wp);
          break;
        }
      }
    }

    if (!matched_waypoints.empty()) {
      RCLCPP_INFO(this->get_logger(), "Rebased matched waypoints (<= %.3fm):", match_radius_);
      for (const auto &wp : matched_waypoints) {
        const double x = wp.first  + robot_base_x_;
        const double y = wp.second + robot_base_y_;
        RCLCPP_INFO(this->get_logger(), "Waypoint: x=%.4f, y=%.4f", x, y);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No waypoints matched for problematic points.");
    }

    // Publish problematic points in base_link
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.header.frame_id = "base_link";

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    mod.resize(problematic_points.size());

    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    for (const auto &p : problematic_points) {
      *ox = p[0]; *oy = p[1]; *oz = p[2];
      ++ox; ++oy; ++oz;  // prefix increments (postfix++ is not defined for these iterators)
    }

    pub_problematic_->publish(out);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthDipDetector>();

  // Build MoveIt interface AFTER node exists
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node, "ur_manipulator");
  node->attach_move_group(move_group);

  node->move_to_center_and_start();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
