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
// These distances follow the URDF joint origins: 0.05 m rail height, 0.05 m
// base-joint height, and 0.40 m to the shoulder pivot; link dimensions below
// come from the shoulder, elbow, wrist-pitch, and wrist-roll joint offsets.
constexpr double shoulderPivotHeight = 0.50;
constexpr double upperArmLength = 0.40;
constexpr double forearmsAndWristOffset = 0.45;
constexpr double toolOffset = 0.04;

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

    // The URDF's rail_joint is a linear translation along world X; the other joints are revolute and rotate about 
    // the local Z axis.
    const double rail_position = joints.at("rail_joint");
    const double base_angle = joints.at("base_joint");
    const double shoulder_angle = joints.at("shoulder_joint");
    const double elbow_angle = joints.at("elbow_joint");
    const double wrist_angle = joints.at("wrist_pitch_joint");
    const double roll_angle = joints.at("wrist_roll_joint");

    // Shoulder, elbow, and wrist pitch all rotate about parallel Y axes. Their
    // angles therefore add to give the orientation of each successive link.
    const double elbow_sum = shoulder_angle + elbow_angle;
    const double tool_pitch = elbow_sum + wrist_angle;

    // Resolve each link's length into radial and vertical components. The
    // 0.45 m segment combines the URDF's 0.40 m elbow offset and 0.05 m
    // wrist-pitch offset; the last 0.04 m is along the tool's local +Z.
    const double arm_radius =
      upperArmLength * std::sin(shoulder_angle) +
      forearmsAndWristOffset * std::sin(elbow_sum) +
      toolOffset * std::sin(tool_pitch);
    const double tool_height =
      shoulderPivotHeight +
      upperArmLength * std::cos(shoulder_angle) +
      forearmsAndWristOffset * std::cos(elbow_sum) +
      toolOffset * std::cos(tool_pitch);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "world";

    // The rail translates along world X. Base yaw rotates the arm's radial
    // reach around world Z; the vertical link components are unaffected.
    pose.pose.position.x = rail_position + arm_radius * std::cos(base_angle);
    pose.pose.position.y = arm_radius * std::sin(base_angle);
    pose.pose.position.z = tool_height;

    // Compose base yaw, total arm pitch, and wrist roll in the URDF joint order.
    // Roll is about the tool axis, so it changes orientation but not the
    // tool-mount origin position calculated above.
    const auto base_rotation = rotation_z(base_angle);
    const auto arm_rotation = rotation_y(tool_pitch);
    const auto roll_rotation = rotation_z(roll_angle);
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
