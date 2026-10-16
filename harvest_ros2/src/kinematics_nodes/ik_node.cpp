#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

// Updated kinematics based on URDF offsets
constexpr double railMin = -0.9;
constexpr double railMax = 0.9;

// Base height: 0.025 (rail) + 0.108 (base) + 0.061 (shoulder) = 0.194 m
constexpr double baseHeight = 0.194;
constexpr double upperArmLength = 0.15;
constexpr double forearmLength = 0.15;

// Tool X offset: 0.14 (actuator mount) + 0.02 (tool mount) = 0.16 m
// Tool Y offset: -0.072 (actuator mount introduces a non-planar shift)
constexpr double toolXOffset = 0.16;
constexpr double toolYOffset = -0.072;

// The solver aims the tool horizontally
constexpr double desiredToolPitch = kPi / 2.0;

bool inside_range(double value, double lower, double upper)
{
  return value >= lower && value <= upper;
}
}  // namespace

class UniversalWristIKNode : public rclcpp::Node
{
public:
  UniversalWristIKNode() : Node("ik_node")
  {
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/harvest_target_pose", 10,
      std::bind(&UniversalWristIKNode::target_callback, this, std::placeholders::_1));
    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);
    RCLCPP_INFO(get_logger(), "IK node started for updated harvest arm.");
  }

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != "world") {
      RCLCPP_ERROR(get_logger(), "Target frame expected 'world'.");
      return;
    }

    const double target_x = msg->pose.position.x;
    const double target_y = msg->pose.position.y;
    const double target_z = msg->pose.position.z;

    const double rail_position = std::clamp(target_x, railMin, railMax);
    const double relative_x = target_x - rail_position; 

    // Because the tool is offset in Y, the arm's radial plane does not intersect the target directly.
    const double target_radius_sq = relative_x * relative_x + target_y * target_y;
    const double y_offset_sq = toolYOffset * toolYOffset;

    if (target_radius_sq < y_offset_sq) {
      RCLCPP_ERROR(get_logger(), "Target is too close to the yaw axis; unreachable due to Y-offset.");
      return;
    }

    // Calculate the radial distance in the arm's pitch plane
    const double planar_radius = std::sqrt(target_radius_sq - y_offset_sq);
    
    // Adjust the base yaw to compensate for the tool's lateral Y-offset
    const double base_angle = std::atan2(target_y, relative_x) - std::atan2(toolYOffset, planar_radius);

    const double wrist_radius = planar_radius - toolXOffset * std::sin(desiredToolPitch);
    const double wrist_z = target_z - baseHeight - toolXOffset * std::cos(desiredToolPitch);

    const double reach_squared = wrist_radius * wrist_radius + wrist_z * wrist_z;
    double cos_elbow = (reach_squared - upperArmLength * upperArmLength - forearmLength * forearmLength) /
                       (2.0 * upperArmLength * forearmLength);

    if (cos_elbow < -1.0 - 1e-9 || cos_elbow > 1.0 + 1e-9) {
      RCLCPP_ERROR(get_logger(), "Target is outside the arm's reachable radial workspace.");
      return;
    }
    cos_elbow = std::clamp(cos_elbow, -1.0, 1.0);

    bool solution_found = false;
    double shoulder_angle = 0.0, elbow_angle = 0.0, wrist_angle = 0.0;

    for (const double elbow_sign : std::array<double, 2>{1.0, -1.0}) {
      const double try_elbow = elbow_sign * std::acos(cos_elbow);
      const double try_shoulder =
        std::atan2(wrist_radius, wrist_z) -
        std::atan2(forearmLength * std::sin(try_elbow),
                   upperArmLength + forearmLength * std::cos(try_elbow));
                   
      const double try_wrist = desiredToolPitch - try_shoulder - try_elbow;

      // URDF limits matching the new model
      if (inside_range(try_shoulder, -1.5708, 1.5708) &&
          inside_range(try_elbow, -2.0, 2.0) &&
          inside_range(try_wrist, -1.5708, 1.5708))
      {
        shoulder_angle = try_shoulder;
        elbow_angle = try_elbow;
        wrist_angle = try_wrist;
        solution_found = true;
        break;
      }
    }

    if (!solution_found) {
      RCLCPP_ERROR(get_logger(), "No valid elbow branch satisfies joint limits.");
      return;
    }

    send_trajectory_point(rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle);
    RCLCPP_INFO(get_logger(),
      "IK target (%.3f, %.3f, %.3f); joints (%.3f, %.3f, %.3f, %.3f, %.3f)",
      target_x, target_y, target_z, rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle);
  }

  void send_trajectory_point(
    double rail_position, double base_angle, double shoulder_angle,
    double elbow_angle, double wrist_angle)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = {
      "rail_joint", "base_joint", "shoulder_joint", "elbow_joint",
      "wrist_pitch_joint", "wrist_roll_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle, 0.0};
    point.time_from_start = rclcpp::Duration::from_seconds(4.0);
    trajectory.points.push_back(point);
    trajectory_pub_->publish(trajectory);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UniversalWristIKNode>());
  rclcpp::shutdown();
  return 0;
}