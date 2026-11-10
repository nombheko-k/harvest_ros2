#include <array>
#include <cmath>
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{
constexpr double robotSpawnHeight = 0.025;
constexpr double graspOffset = 0.045;
constexpr std::array<const char *, 6> jointNames = {
  "rail_joint", "base_joint", "shoulder_joint", "elbow_joint",
  "wrist_pitch_joint", "wrist_roll_joint"};

// 4x4 Matrix Structure for Direct Homogeneous Transformations
struct Matrix4
{
  double m[4][4];

  // create 4x4 identity matrix with main diagonal of 1.0
  static Matrix4 identity()
  {
    Matrix4 res;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) res.m[i][j] = (i == j) ? 1.0 : 0.0;
    return res;
  }

  // performs standard 4x4 matrix multiplication to combine geometric transformations
  Matrix4 operator*(const Matrix4 & rhs) const
  {
    Matrix4 res;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        res.m[i][j] = 0.0;
        for (int k = 0; k < 4; ++k) {
          res.m[i][j] += m[i][k] * rhs.m[k][j];
        }
      }
    }
    return res;
  }

  // Constructs a transformation matrix representing pure translation along X,Y,Z
  static Matrix4 translation(double x, double y, double z)
  {
    Matrix4 res = identity();
    res.m[0][3] = x;
    res.m[1][3] = y;
    res.m[2][3] = z;
    return res;
  }

  // Constructs a rotaion matrix around the x-axis
  static Matrix4 rotationX(double roll)
  {
    Matrix4 res = identity();
    double c = std::cos(roll), s = std::sin(roll);
    res.m[1][1] = c; res.m[1][2] = -s;
    res.m[2][1] = s; res.m[2][2] = c;
    return res;
  }

  // around y-axis
  static Matrix4 rotationY(double pitch)
  {
    Matrix4 res = identity();
    double c = std::cos(pitch), s = std::sin(pitch);
    res.m[0][0] = c; res.m[0][2] = s;
    res.m[2][0] = -s; res.m[2][2] = c;
    return res;
  }

  // around z-axis
  static Matrix4 rotationZ(double yaw)
  {
    Matrix4 res = identity();
    double c = std::cos(yaw), s = std::sin(yaw);
    res.m[0][0] = c; res.m[0][1] = -s;
    res.m[1][0] = s; res.m[1][1] = c;
    return res;
  }

  // Combines roll, pitch and yaw rotations into a single matrix using Z-Y-X convention 
  static Matrix4 rpy(double roll, double pitch, double yaw)
  {
    return rotationZ(yaw) * rotationY(pitch) * rotationX(roll);
  }
};

// Convert Rotation Matrix to Quaternion
geometry_msgs::msg::Quaternion matrix_to_quaternion(const Matrix4 & mat)
{
  geometry_msgs::msg::Quaternion q;
  double trace = mat.m[0][0] + mat.m[1][1] + mat.m[2][2];

  // numerical stability handling for conversion using the matrix trace
  if (trace > 0.0) {
    double s = 0.5 / std::sqrt(trace + 1.0);
    q.w = 0.25 / s;
    q.x = (mat.m[2][1] - mat.m[1][2]) * s;
    q.y = (mat.m[0][2] - mat.m[2][0]) * s;
    q.z = (mat.m[1][0] - mat.m[0][1]) * s;
  } else {
    if (mat.m[0][0] > mat.m[1][1] && mat.m[0][0] > mat.m[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + mat.m[0][0] - mat.m[1][1] - mat.m[2][2]);
      q.w = (mat.m[2][1] - mat.m[1][2]) / s;
      q.x = 0.25 * s;
      q.y = (mat.m[0][1] + mat.m[1][0]) / s;
      q.z = (mat.m[0][2] + mat.m[2][0]) / s;
    } else if (mat.m[1][1] > mat.m[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + mat.m[1][1] - mat.m[0][0] - mat.m[2][2]);
      q.w = (mat.m[0][2] - mat.m[2][0]) / s;
      q.x = (mat.m[0][1] + mat.m[1][0]) / s;
      q.y = 0.25 * s;
      q.z = (mat.m[1][2] + mat.m[2][1]) / s;
    } else {
      double s = 2.0 * std::sqrt(1.0 + mat.m[2][2] - mat.m[0][0] - mat.m[1][1]);
      q.w = (mat.m[1][0] - mat.m[0][1]) / s;
      q.x = (mat.m[0][2] + mat.m[2][0]) / s;
      q.y = (mat.m[1][2] + mat.m[2][1]) / s;
      q.z = 0.25 * s;
    }
  }
  return q;
}

}  // namespace

// ROS2 node dedicated to computing the FK chain
class FKNode : public rclcpp::Node
{
public:
  FKNode() : Node("fk_node")
  {
    // subscribe to incoming robot joint states
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FKNode::joint_callback, this, std::placeholders::_1));
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/harvest_target_pose", 10,
      std::bind(&FKNode::target_callback, this, std::placeholders::_1));
    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10,
      std::bind(&FKNode::trajectory_callback, this, std::placeholders::_1));
    // Keep the tool-origin output and publish the finger-midpoint grasp point separately.
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/calculated_end_effector_pose", 10);
    grasp_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/calculated_grasp_point_pose", 10);
    RCLCPP_INFO(get_logger(), "FK node started with tool and grasp-point outputs.");
  }

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != "world") {
      RCLCPP_ERROR(get_logger(), "FK target comparison requires the 'world' frame.");
      return;
    }
    latest_target_ = *msg;
    have_target_ = true;
    // A previous crop's endpoint must not be compared with the newly selected berry.
    have_commanded_endpoint_ = false;
    log_commanded_endpoint_error();
  }

  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring an empty trajectory for FK endpoint validation.");
      return;
    }

    const auto & endpoint = msg->points.back();
    for (size_t joint_index = 0; joint_index < jointNames.size(); ++joint_index) {
      const auto joint = std::find(
        msg->joint_names.begin(), msg->joint_names.end(), jointNames[joint_index]);
      if (joint == msg->joint_names.end()) {
        RCLCPP_ERROR(
          get_logger(), "Cannot validate FK endpoint: trajectory is missing joint '%s'.",
          jointNames[joint_index]);
        return;
      }
      const size_t trajectory_index =
        static_cast<size_t>(std::distance(msg->joint_names.begin(), joint));
      if (trajectory_index >= endpoint.positions.size()) {
        RCLCPP_ERROR(
          get_logger(), "Cannot validate FK endpoint: no position for joint '%s'.",
          jointNames[joint_index]);
        return;
      }
      commanded_endpoint_joints_[joint_index] = endpoint.positions[trajectory_index];
    }
    have_commanded_endpoint_ = true;
    log_commanded_endpoint_error();
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Do not publish a plausible-looking pose when feedback is missing a joint.
    std::unordered_map<std::string, double> joints;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->position.size()) {
        joints[msg->name[i]] = msg->position[i];
      }
    }
    std::array<double, jointNames.size()> joint_positions{};
    for (size_t index = 0; index < jointNames.size(); ++index) {
      const auto joint = joints.find(jointNames[index]);
      if (joint == joints.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for feedback for all six arm joints before computing FK.");
        return;
      }
      joint_positions[index] = joint->second;
    }

    const Matrix4 T_total = forward_transform(joint_positions);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "world";
    pose.pose.position.x = T_total.m[0][3];
    pose.pose.position.y = T_total.m[1][3];
    pose.pose.position.z = T_total.m[2][3];
    pose.pose.orientation = matrix_to_quaternion(T_total);

    geometry_msgs::msg::PoseStamped grasp_pose = pose;
    // Finger midpoint lies 45 mm along the tool's local +X axis from the tool mount.
    grasp_pose.pose.position.x += graspOffset * T_total.m[0][0];
    grasp_pose.pose.position.y += graspOffset * T_total.m[1][0];
    grasp_pose.pose.position.z += graspOffset * T_total.m[2][0];

    // Publish both tool origin and the actual point IK aims at for grasping.
    pose_pub_->publish(pose);
    grasp_pub_->publish(grasp_pose);
    if (have_target_) {
      const double dx = grasp_pose.pose.position.x - latest_target_.pose.position.x;
      const double dy = grasp_pose.pose.position.y - latest_target_.pose.position.y;
      const double dz = grasp_pose.pose.position.z - latest_target_.pose.position.z;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Measured FK tracking error (nonzero while moving is expected): %.4f m "
        "(x %.4f, y %.4f, z %.4f)",
        std::sqrt(dx * dx + dy * dy + dz * dz), dx, dy, dz);
    }
  }

  Matrix4 forward_transform(const std::array<double, jointNames.size()> & joints) const
  {
    // This chain mirrors the URDF origins, joint axes, and fixed tool offsets.
    const Matrix4 T_rail =
      Matrix4::translation(joints[0], 0.0, robotSpawnHeight + 0.025);
    const Matrix4 T_base =
      Matrix4::translation(0.0, 0.0, 0.108) * Matrix4::rotationZ(joints[1]);
    const Matrix4 T_shoulder =
      Matrix4::translation(0.0, 0.0, 0.061) *
      Matrix4::rpy(0.0, -M_PI_2, 0.0) * Matrix4::rotationY(joints[2]);
    const Matrix4 T_elbow =
      Matrix4::translation(0.15, 0.0, 0.0) * Matrix4::rotationY(joints[3]);
    const Matrix4 T_pitch =
      Matrix4::translation(0.15, 0.0, 0.0) * Matrix4::rotationY(joints[4]);
    const Matrix4 T_roll_actuator = Matrix4::translation(0.14, -0.072, 0.0);
    const Matrix4 T_roll_joint =
      Matrix4::translation(0.02, 0.0, 0.0) * Matrix4::rotationX(joints[5]);
    return T_rail * T_base * T_shoulder * T_elbow * T_pitch *
           T_roll_actuator * T_roll_joint;
  }

  void log_commanded_endpoint_error()
  {
    if (!have_target_ || !have_commanded_endpoint_) {
      return;
    }

    const Matrix4 transform = forward_transform(commanded_endpoint_joints_);
    const double grasp_x = transform.m[0][3] + graspOffset * transform.m[0][0];
    const double grasp_y = transform.m[1][3] + graspOffset * transform.m[1][0];
    const double grasp_z = transform.m[2][3] + graspOffset * transform.m[2][0];
    const double dx = grasp_x - latest_target_.pose.position.x;
    const double dy = grasp_y - latest_target_.pose.position.y;
    const double dz = grasp_z - latest_target_.pose.position.z;

    // This endpoint check isolates the IK/FK geometry from the arm's in-flight tracking error.
    RCLCPP_INFO(
      get_logger(),
      "Commanded endpoint FK grasp error: %.4f m (x %.4f, y %.4f, z %.4f)",
      std::sqrt(dx * dx + dy * dy + dz * dz), dx, dy, dz);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pub_;
  geometry_msgs::msg::PoseStamped latest_target_;
  std::array<double, jointNames.size()> commanded_endpoint_joints_{};
  bool have_target_ = false;
  bool have_commanded_endpoint_ = false;
};

// Node initialisation entry point
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FKNode>());
  rclcpp::shutdown();
  return 0;
}