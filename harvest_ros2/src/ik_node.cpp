#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

// These limits and link distances match the rail and arm URDF. The 0.45 m
// segment is the 0.40 m elbow-joint offset plus the 0.05 m wrist-pitch offset.
constexpr double railMin = -0.9; // Rail lower limit is -0.9 m; upper limit is 0.9 m.
constexpr double railMax = 0.9;
constexpr double baseHeight = 0.133;
constexpr double upperArmLength = 0.150;
constexpr double forearmsAndWristOffset = 0.150;
constexpr double toolOffset = 0.1755;

// The solver aims the tool horizontally and outward from the base in the arm's
// radial plane. In this URDF, zero combined pitch points straight up.
constexpr double desiredToolPitch = kPi / 2.0;

// Keep candidate joint values inside the corresponding URDF position limits.
bool inside_range(double value, double lower, double upper)
{
  return value >= lower && value <= upper;
}
}  // namespace

// enum definition for the harvest state machine
enum class HarvestState {
  TRACKING_TARGET,
  READY_TO_TWIST,
  TWISTING,
  HARVEST_COMPLETE
};

class UniversalWristIKNode : public rclcpp::Node
{
public:
  UniversalWristIKNode()
  : Node("ik_node"), current_state_(HarvestState::TRACKING_TARGET)
  {
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/harvest_target_pose", 10,
      std::bind(&UniversalWristIKNode::target_callback, this, std::placeholders::_1));

      // Subscribe to current joint states to verify when we reach the target
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&UniversalWristIKNode::joint_callback, this, std::placeholders::_1));

    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);

    RCLCPP_INFO(get_logger(), "IK Node started with active Harvest State Machine.");
  }

private:
  HarvestState current_state_;
  bool has_target_ = false; // <-- safety latch
  std::vector<double> target_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0}; // rail, base, shoulder, elbow, pitch
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != "world") {
      RCLCPP_ERROR(
        get_logger(), "Target frame is '%s'; expected 'world'. Transform the target first.",
        msg->header.frame_id.c_str());
      return;
    }

    const double target_x = msg->pose.position.x;
    const double target_y = msg->pose.position.y;
    const double target_z = msg->pose.position.z;
    if (!std::isfinite(target_x) || !std::isfinite(target_y) || !std::isfinite(target_z)) {
      RCLCPP_ERROR(get_logger(), "Target position contains a non-finite coordinate.");
      return;
    }

    const double rail_position = std::clamp(target_x, railMin, railMax);
    const double relative_x = target_x - rail_position;

    const double target_radius = std::hypot(relative_x, target_y);
    const double base_angle = std::atan2(target_y, relative_x);

    const double wrist_radius =
      target_radius - toolOffset * std::sin(desiredToolPitch);
    const double wrist_z =
      target_z - baseHeight - toolOffset * std::cos(desiredToolPitch);

    const double reach_squared = wrist_radius * wrist_radius + wrist_z * wrist_z;
    double cos_elbow =
      (reach_squared - upperArmLength * upperArmLength -
      forearmsAndWristOffset * forearmsAndWristOffset) /
      (2.0 * upperArmLength * forearmsAndWristOffset);

    if (cos_elbow < -1.0 - 1e-9 || cos_elbow > 1.0 + 1e-9) {
      RCLCPP_ERROR(
        get_logger(), "Target (%.3f, %.3f, %.3f) is outside the arm's reachable workspace.",
        target_x, target_y, target_z);
      return;
    }
    cos_elbow = std::clamp(cos_elbow, -1.0, 1.0);

    bool solution_found = false;
    double shoulder_angle = 0.0;
    double elbow_angle = 0.0;
    double wrist_angle = 0.0;

    for (const double elbow_sign : std::array<double, 2>{1.0, -1.0}) {
      const double try_elbow = elbow_sign * std::acos(cos_elbow);

      const double try_shoulder =
        std::atan2(wrist_radius, wrist_z) -
        std::atan2(
          forearmsAndWristOffset * std::sin(try_elbow),
          upperArmLength + forearmsAndWristOffset * std::cos(try_elbow));

      const double try_wrist = desiredToolPitch - try_shoulder - try_elbow;

      if (inside_range(try_shoulder, -1.57, 1.57) &&
          inside_range(try_elbow, -2.0, 2.0) &&
          inside_range(try_wrist, -1.57, 1.57)) {
        shoulder_angle = try_shoulder;
        elbow_angle = try_elbow;
        wrist_angle = try_wrist;
        solution_found = true;
        break;
      }
    }

    if (!solution_found) {
      RCLCPP_ERROR(get_logger(), "No joint limit solution found.");
      return;
    }

    // State Machine logic: Only update approach target if we are tracking
    if (current_state_ == HarvestState::TRACKING_TARGET) {
      target_joints_ = {rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle};
      has_target_ = true; // <-- Lock the target in!

      // Send approach command with 0.0 wrist roll
      send_trajectory_point(
        rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle, 0.0, 4.0);

      RCLCPP_INFO(
        get_logger(),
        "IK target (%.3f, %.3f, %.3f); joints rail/base/shoulder/elbow/wrist_pitch "
        "(%.3f, %.3f, %.3f, %.3f, %.3f) rad",
        target_x, target_y, target_z, rail_position, base_angle,
        shoulder_angle, elbow_angle, wrist_angle);
    }
  }

  void send_trajectory_point(
    double rail_position, double base_angle, double shoulder_angle,
    double elbow_angle, double wrist_angle, double wrist_roll_angle, double duration_sec)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = {
      "rail_joint", "base_joint", "shoulder_joint", "elbow_joint",
      "wrist_pitch_joint", "wrist_roll_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {
      rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle, wrist_roll_angle};
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    trajectory.points.push_back(point);
    trajectory_pub_->publish(trajectory);
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
     // Ignore checking arrivals if we haven't actively received a target coordinates yet
    if (!has_target_ || current_state_ != HarvestState::TRACKING_TARGET) return;

    // Map joint names to their current positions
    std::unordered_map<std::string, double> current_positions;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->position.size()) {
        current_positions[msg->name[i]] = msg->position[i];
      }
    }

    // Define the joint order matching target_joints_
    std::vector<std::string> joint_names = {
      "rail_joint", "base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint"};

    bool target_reached = true;
    constexpr double kPosTolerance = 0.02;

    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (current_positions.find(joint_names[i]) == current_positions.end()) {
        target_reached = false;
        break;
      }
      const double error = std::abs(current_positions[joint_names[i]] - target_joints_[i]);
      if (error > kPosTolerance) {
        target_reached = false;
        break;
      }
    }

    // Trigger Sequence: Target Locked and Reached
    if (target_reached) {
      current_state_ = HarvestState::READY_TO_TWIST;
      RCLCPP_INFO(get_logger(), "Target Locked and Reached. Activating harvest twist sequence!");
      execute_harvest_twist();
    }
  }

  void execute_harvest_twist()
  {
    current_state_ = HarvestState::TWISTING;

    // Maintain the exact same position for the arm joints, but actuate wrist roll to 90 degrees (kPi/2)
    // A shorter time duration (e.g., 1.5 seconds) makes the twist snappy for the harvest detaching motion
    send_trajectory_point(
      target_joints_[0], target_joints_[1], target_joints_[2],
      target_joints_[3], target_joints_[4], kPi / 2.0, 1.5);

    current_state_ = HarvestState::HARVEST_COMPLETE;
    RCLCPP_INFO(get_logger(), "Harvest twist action commanded.");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UniversalWristIKNode>());
  rclcpp::shutdown();
  return 0;
}
