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
// Link Dimensions
constexpr double shoulderPivotHeight = 0.194;
constexpr double upperArmLength = 0.150;
constexpr double forearmsAndWristOffset = 0.150;
constexpr double toolOffset = 0.1755;

// MATRIX AND MATH HELPERS
// Custom 4x4 Matrix structure to handle homogeneous transformations 
// (Rotation + Position) without external dependencies like Eigen.
// 'dh_transform' generates the standard matrix from alpha, a, d, theta parameters.
// 'matrix_to_quaternion' converts the resulting rotation matrix into the ROS message format.

// 4x4 Matrix Structure for Homogeneous Transformations
struct Matrix4
{
  double m[4][4];

  static Matrix4 identity()
  {
    Matrix4 res;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) res.m[i][j] = (i == j) ? 1.0 : 0.0;
    return res;
  }

  // Matrix Multiplication
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
};

// Generate standard DH Matrix
// theta: rotation about Z
// d: translation along Z
// a: translation along X (common normal)
// alpha: rotation about X
Matrix4 dh_transform(double theta, double d, double a, double alpha)
{
  Matrix4 res;
  double ct = std::cos(theta);
  double st = std::sin(theta);
  double ca = std::cos(alpha);
  double sa = std::sin(alpha);

  res.m[0][0] = ct;
  res.m[0][1] = -st * ca;
  res.m[0][2] = st * sa;
  res.m[0][3] = a * ct;

  res.m[1][0] = st;
  res.m[1][1] = ct * ca;
  res.m[1][2] = -ct * sa;
  res.m[1][3] = a * st;

  res.m[2][0] = 0.0;
  res.m[2][1] = sa;
  res.m[2][2] = ca;
  res.m[2][3] = d;

  res.m[3][0] = 0.0;
  res.m[3][1] = 0.0;
  res.m[3][2] = 0.0;
  res.m[3][3] = 1.0;

  return res;
}

// Convert Rotation Matrix (top-left 3x3) to Quaternion
geometry_msgs::msg::Quaternion matrix_to_quaternion(const Matrix4 & mat)
{
  geometry_msgs::msg::Quaternion q;
  double trace = mat.m[0][0] + mat.m[1][1] + mat.m[2][2];

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

class FKNode : public rclcpp::Node
{
public:
  FKNode()
  : Node("fk_node")
  {
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FKNode::joint_callback, this, std::placeholders::_1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/calculated_end_effector_pose", 10);
    RCLCPP_INFO(
      get_logger(),
      "FK node started using DH Parameters.");
  }

// JOINT STATE PARSING
// Parse the incoming JointState message. We use a map to ensure we find 
// every specific joint name we need, regardless of the order they appear 
// in the message array. If any required joint is missing, we abort to avoid 
// calculating an invalid pose.

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const std::unordered_map<std::string, double> required_joints = {
      {"rail_joint", 0.0},
      {"base_joint", 0.0},
      {"shoulder_joint", 0.0},
      {"elbow_joint", 0.0},
      {"wrist_pitch_joint", 0.0},
      {"wrist_roll_joint", 0.0}};
    auto joints = required_joints;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) continue;
      const auto joint = joints.find(msg->name[i]);
      if (joint != joints.end()) {
        joint->second = msg->position[i];
      }
    }

    // Extract joint values
    const double q_rail = joints.at("rail_joint");
    const double q_base = joints.at("base_joint");
    const double q_shoulder = joints.at("shoulder_joint");
    const double q_elbow = joints.at("elbow_joint");
    const double q_pitch = joints.at("wrist_pitch_joint");
    const double q_roll = joints.at("wrist_roll_joint");

    // DH PARAMETER LOGIC
    // Note: M_PI/2 offsets are added to shoulder/elbow/pitch theta because
    // the URDF/IK defines 0.0 as "Up" (along Z), whereas standard DH defines 
    // 0.0 as "along the link" (along X).

    // 0. World to Rail Transform (Prismatic):
    // The rail translates along the World X axis. Since DH usually assumes 
    // revolute joints, we treat this as a base frame translation.
    Matrix4 T_rail = Matrix4::identity();
    T_rail.m[0][3] = q_rail; 

    // 1. Base Joint (Link 1):
    // Theta = base_angle. 
    // Alpha = 90 degrees (PI/2) rotates the Z-axis to point horizontally, 
    // setting the coordinate system up for the vertical shoulder arm.
    Matrix4 T_base = dh_transform(q_base, shoulderPivotHeight, 0.0, M_PI_2);

    // 2. Shoulder Joint (Link 2):
    // We add PI/2 to the shoulder angle because standard DH assumes 0.0 is horizontal,
    // but our robot model/IK defines 0.0 as vertical (up).
    // 'a' = upperArmLength (length of the link).
    Matrix4 T_shoulder = dh_transform(M_PI_2 - q_shoulder, 0.0, upperArmLength, 0.0);

    // 3. Elbow Joint (Link 3):
    // Parallel to the shoulder, so Alpha is 0. 
    // 'a' = forearmsAndWristOffset represents the forearm length.
    Matrix4 T_elbow = dh_transform(-q_elbow, 0.0, forearmsAndWristOffset, 0.0);

    // 4. Wrist Pitch (Link 4):
    // Alpha = -90 degrees (-PI/2). This rotates the Z-axis to point forward, 
    // aligned with the tool tip. This ensures the subsequent 'Roll' joint 
    // rotates around the tool's directional vector.
    Matrix4 T_pitch = dh_transform(-q_pitch, 0.0, toolOffset, -M_PI_2);

    // 5. Wrist Roll: Rotates around the tool axis (Z).
    Matrix4 T_roll = dh_transform(q_roll, 0.0, 0.0, 0.0);

    // Compute Forward Kinematics Chain
    // Multiply all transforms in order from Base -> Tip to get the total 
    Matrix4 T_total = T_rail * T_base * T_shoulder * T_elbow * T_pitch * T_roll;

    // --- Output ---
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "world";

    // pose of the tool mount relative to the World frame.
    // Position is extracted from the last column (0,3), (1,3), (2,3).
    // Orientation is extracted from the top-left 3x3 rotation matrix.
    pose.pose.position.x = T_total.m[0][3]; // maps to X
    pose.pose.position.y = T_total.m[1][3]; // maps to Y
    pose.pose.position.z = T_total.m[2][3]; // maps to Z
    pose.pose.orientation = matrix_to_quaternion(T_total);

    pose_pub_->publish(pose);
    RCLCPP_INFO(
      get_logger(), "FK (DH) -> X: %.4f, Y: %.4f, Z: %.4f",
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
