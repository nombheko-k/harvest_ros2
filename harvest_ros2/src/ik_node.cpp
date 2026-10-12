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

// These limits and link distances match the rail and arm URDF. The 0.45 m
// segment is the 0.40 m elbow-joint offset plus the 0.05 m wrist-pitch offset.
constexpr double railMin = -0.9; // Rail lower limit is -0.9 m; URDF upper limit is 0.9 m.
constexpr double railMax = 0.9;
constexpr double baseHeight = 0.50;
constexpr double upperArmLength = 0.40;
constexpr double forearmsAndWristOffset = 0.45;
constexpr double toolOffset = 0.04;

// The solver aims the tool horizontally and outward from the base in the arm's
// radial plane. In this URDF, zero combined pitch points straight up.
constexpr double desiredToolPitch = kPi / 2.0;

// Keep candidate joint values inside the corresponding URDF position limits.
bool inside_range(double value, double lower, double upper)
{
  return value >= lower && value <= upper;
}
}  // namespace

class UniversalWristIKNode : public rclcpp::Node
{
public:
  UniversalWristIKNode()
  : Node("ik_node")
  {
    // Targets are PoseStamped messages in world coordinates; commands go to
    // the configured six-joint trajectory controller.
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/harvest_target_pose", 10,
      std::bind(&UniversalWristIKNode::target_callback, this, std::placeholders::_1));
    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);
    RCLCPP_INFO(
      get_logger(),
      "IK node started for end_effector_tool_mount; target frame must be 'world'.");
  }

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // The equations below operate in the robot's world frame. Do not silently
    // interpret coordinates from another frame as world coordinates.
    if (msg->header.frame_id != "world") {
      RCLCPP_ERROR(
        get_logger(), "Target frame is '%s'; expected 'world'. Transform the target first.",
        msg->header.frame_id.c_str());
      return;
    }

    const double target_x = msg->pose.position.x;
    const double target_y = msg->pose.position.y;
    const double target_z = msg->pose.position.z;
    if (!std::isfinite(target_x) || !std::isfinite(target_y) ||
      !std::isfinite(target_z))
    {
      RCLCPP_ERROR(get_logger(), "Target position contains a non-finite coordinate.");
      return;
    }

    // The rail only translates along world X. Put it directly beneath the
    // target's X coordinate when within travel; clamp at the rail endpoints.
    // The remaining target offset is handled by base yaw and the arm links.
    const double rail_position = std::clamp(target_x, railMin, railMax);
    const double relative_x = target_x - rail_position; 

    // Convert the remaining world X/Y offset into the arm's radial plane:
    // radius is horizontal distance from the base yaw axis, angle is its yaw.
    const double target_radius = std::hypot(relative_x, target_y);
    const double base_angle = std::atan2(target_y, relative_x); // this is base angle found in our model

    // Back out the final 0.04 m tool-mount offset to find the wrist point the
    // two-link arm must reach. Because the requested tool pitch is horizontal,
    // that offset contributes radially and contributes no vertical distance.
    const double wrist_radius =
      target_radius - toolOffset * std::sin(desiredToolPitch); // basically finding the wrist centre like in our model
    const double wrist_z =
      target_z - baseHeight - toolOffset * std::cos(desiredToolPitch);

    // Apply the cosine rule to the 0.40 m and 0.45 m links. cos_elbow must be
    // between -1 and 1 for a real arm configuration; tiny round-off outside
    // that range is tolerated and clamped below.
    const double reach_squared = wrist_radius * wrist_radius + wrist_z * wrist_z; // horizontal distance from base to wrist centre
    double cos_elbow =
      (reach_squared - upperArmLength * upperArmLength -
      forearmsAndWristOffset * forearmsAndWristOffset) /
      (2.0 * upperArmLength * forearmsAndWristOffset); // use law of cosines to find the elbow angle like in model

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

    // The same target can have two elbow configurations. Try both signs and
    // select the first shoulder/elbow/wrist combination within URDF limits.
    for (const double elbow_sign : std::array<double, 2>{1.0, -1.0}) {
      const double try_elbow = elbow_sign * std::acos(cos_elbow);

      // Resolve the shoulder angle from the target direction minus the angle
      // contributed by the forearm link (standard planar two-link IK).
      const double try_shoulder =
        std::atan2(wrist_radius, wrist_z) -
        std::atan2(
        forearmsAndWristOffset * std::sin(try_elbow),
        upperArmLength + forearmsAndWristOffset * std::cos(try_elbow));
      // The three pitch joints add together in this URDF, so choose wrist
      // pitch to produce the desired total tool pitch.
      const double try_wrist =
        desiredToolPitch - try_shoulder - try_elbow;

      if (inside_range(try_shoulder, -1.57, 1.57) &&  // between -90 and +90 degrees
        inside_range(try_elbow, -2.0, 2.0) && // between -115 and +115 degrees
        inside_range(try_wrist, -1.57, 1.57))
      {
        shoulder_angle = try_shoulder;
        elbow_angle = try_elbow;
        wrist_angle = try_wrist;
        solution_found = true;
        break;
      }
    }

    if (!solution_found) {
      RCLCPP_ERROR(
        get_logger(),
        "Target is geometrically reachable, but no elbow branch satisfies the URDF joint limits.");
      return;
    }

    // The joint angles are now known; send them to the trajectory controller.
    send_trajectory_point(
      rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle);
    RCLCPP_INFO(
      get_logger(),
      "IK target (%.3f, %.3f, %.3f); joints rail/base/shoulder/elbow/wrist_pitch "
      "(%.3f, %.3f, %.3f, %.3f, %.3f) rad",
      target_x, target_y, target_z, rail_position, base_angle,
      shoulder_angle, elbow_angle, wrist_angle);
  }

  void send_trajectory_point(
    double rail_position, double base_angle, double shoulder_angle,
    double elbow_angle, double wrist_angle)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();

    // The positions array must follow this exact name order. Wrist roll is
    // zero here because it does not change the tool-mount position.
    trajectory.joint_names = {
      "rail_joint", "base_joint", "shoulder_joint", "elbow_joint",
      "wrist_pitch_joint", "wrist_roll_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {
      rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle, 0.0};
    // The rail spans 1.8 m and is limited to 0.5 m/s, so four seconds is
    // sufficient for even a full-range traverse (minimum 3.6 seconds).
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
