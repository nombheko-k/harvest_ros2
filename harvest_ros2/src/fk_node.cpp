#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

// The carriage frame sits at the rail's top face (25 mm); the base yaw is
// 108 mm above it, and the shoulder pivot is another 96 mm above the yaw.
constexpr double shoulderPivotHeight = 0.025 + 0.108 + 0.096;
constexpr double armLinkLength = 0.15;
constexpr double toolOffset = 0.02;

// Quaternion composition is used to report the tool orientation in world.
// Multiplication order matters: the right-hand rotation is applied first.
geometry_msgs::msg::Quaternion multiply(
  const geometry_msgs::msg::Quaternion & lhs,
  const geometry_msgs::msg::Quaternion & rhs)
{
  geometry_msgs::msg::Quaternion result;
  result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
  result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
  result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
  result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
  return result;
}

geometry_msgs::msg::Quaternion rotation_z(double angle)
{
  // Unit quaternion for a rotation about the local/world Z axis.
  geometry_msgs::msg::Quaternion result;
  result.z = std::sin(angle / 2.0);
  result.w = std::cos(angle / 2.0);
  return result;
}

geometry_msgs::msg::Quaternion rotation_y(double angle)
{
  // Unit quaternion for a rotation about the local Y axis.
  geometry_msgs::msg::Quaternion result;
  result.y = std::sin(angle / 2.0);
  result.w = std::cos(angle / 2.0);
  return result;
}

geometry_msgs::msg::Quaternion rotation_x(double angle)
{
  // The wrist-roll actuator turns about the arm's local longitudinal axis.
  geometry_msgs::msg::Quaternion result;
  result.x = std::sin(angle / 2.0);
  result.w = std::cos(angle / 2.0);
  return result;
}
}  // namespace

class FKNode : public rclcpp::Node
{
public:
  FKNode()
  : Node("fk_node")
  {
    // JointState arrays are looked up by name below, not assumed to have a
    // fixed order; the result is the tool-mount pose in the world frame.
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FKNode::joint_callback, this, std::placeholders::_1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/calculated_end_effector_pose", 10);
    RCLCPP_INFO(
      get_logger(),
      "Forward kinematics node started; calculating end_effector_tool_mount from the URDF chain.");
  }

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Start with the required set so we can detect if any expected joint is
    // absent from a message before publishing a pose.
    const std::unordered_map<std::string, double> required_joints = {
      {"rail_joint", 0.0},
      {"base_joint", 0.0},
      {"shoulder_joint", 0.0},
      {"elbow_joint", 0.0},
      {"wrist_pitch_joint", 0.0},
      {"wrist_roll_joint", 0.0}};
    auto joints = required_joints;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      // JointState requires one position per name. Reject malformed messages
      // rather than reading past the position array.
      if (i >= msg->position.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring JointState with fewer positions than joint names.");
        return;
      }
      const auto joint = joints.find(msg->name[i]);
      if (joint != joints.end()) {
        // Ignore unrelated joints (for example, gripper fingers).
        joint->second = msg->position[i];
      }
    }

    // Do not substitute zero for missing arm joints: that would publish a
    // plausible-looking pose for a joint configuration the robot never had.
    for (const auto & joint : joints) {
      bool found = false;
      for (const auto & name : msg->name) {
        if (name == joint.first) {
          found = true;
          break;
        }
      }
      if (!found) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring JointState; required joint '%s' is missing.", joint.first.c_str());
        return;
      }
    }

    // Rail travel and the base yaw are world-X and world-Z motions respectively;
    // the three arm pitches use local Y and wrist roll uses the local X axis.
    const double rail_position = joints.at("rail_joint");
    const double base_angle = joints.at("base_joint");
    const double shoulder_angle = joints.at("shoulder_joint");
    const double elbow_angle = joints.at("elbow_joint");
    const double wrist_angle = joints.at("wrist_pitch_joint");
    const double roll_angle = joints.at("wrist_roll_joint");

    // Each link extends along local +X. The shoulder frame contributes the
    // initial -90 degree pitch, so zero joint angles yield a vertical arm.
    const double elbow_sum = shoulder_angle + elbow_angle;
    const double tool_pitch = elbow_sum + wrist_angle;
    const double shoulder_pitch = shoulder_angle - kPi / 2.0;
    const double elbow_pitch = elbow_sum - kPi / 2.0;
    const double wrist_pitch = tool_pitch - kPi / 2.0;
    const double arm_radius =
      armLinkLength * std::cos(shoulder_pitch) +
      armLinkLength * std::cos(elbow_pitch) +
      (armLinkLength + toolOffset) * std::cos(wrist_pitch);
    const double tool_height =
      shoulderPivotHeight -
      armLinkLength * std::sin(shoulder_pitch) -
      armLinkLength * std::sin(elbow_pitch) -
      (armLinkLength + toolOffset) * std::sin(wrist_pitch);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "world";

    // The rail translates along world X. Base yaw rotates the arm's radial
    // reach around world Z; the vertical link components are unaffected.
    pose.pose.position.x = rail_position + arm_radius * std::cos(base_angle);
    pose.pose.position.y = arm_radius * std::sin(base_angle);
    pose.pose.position.z = tool_height;

    // Compose yaw, the cumulative pitch, and the longitudinal wrist roll in
    // the same order as the URDF joints.
    const auto base_rotation = rotation_z(base_angle);
    const auto arm_rotation = rotation_y(wrist_pitch);
    const auto roll_rotation = rotation_x(roll_angle);
    pose.pose.orientation =
      multiply(multiply(base_rotation, arm_rotation), roll_rotation);

    pose_pub_->publish(pose);
    RCLCPP_INFO(
      get_logger(), "FK tool mount in world -> X: %.4f, Y: %.4f, Z: %.4f",
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FKNode>());
  rclcpp::shutdown();
  return 0;
}
