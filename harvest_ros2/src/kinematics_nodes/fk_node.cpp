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
    // Publisher for the final evaluated end-effector coordinates
      pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/calculated_end_effector_pose", 10);
    RCLCPP_INFO(get_logger(), "FK node started using URDF-mapped transforms.");
  }

private:
// callback triggered when new joint state message arrives
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    //define and initialise the target joint configuration
    const std::unordered_map<std::string, double> required_joints = {
      {"rail_joint", 0.0}, {"base_joint", 0.0}, {"shoulder_joint", 0.0},
      {"elbow_joint", 0.0}, {"wrist_pitch_joint", 0.0}, {"wrist_roll_joint", 0.0}};
    auto joints = required_joints;

    // safely extract relevant joint states from incoming data arrays based on matching names
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) continue; // Bounds to guard agianst mismatched arrays
      auto joint = joints.find(msg->name[i]);
      if (joint != joints.end()) {
        joint->second = msg->position[i];
      }
    }

    // Assign mapped joint angles to corresponding local state tracking variables
    const double q_rail = joints.at("rail_joint");
    const double q_base = joints.at("base_joint");
    const double q_shoulder = joints.at("shoulder_joint");
    const double q_elbow = joints.at("elbow_joint");
    const double q_pitch = joints.at("wrist_pitch_joint");
    const double q_roll = joints.at("wrist_roll_joint");

    // Kinematic Chain mapped directly from the URDF
    
    // 0. World to Base Link via Rail (Prismatic X)
    Matrix4 T_rail = Matrix4::translation(q_rail, 0.0, 0.025);

    // 1. Base Joint (Revolute Z)
    Matrix4 T_base = Matrix4::translation(0.0, 0.0, 0.108) * Matrix4::rotationZ(q_base);

    // 2. Shoulder Joint (Revolute Y, initial pitch offset points arm +Z)
    Matrix4 T_shoulder = Matrix4::translation(0.0, 0.0, 0.061) * 
                         Matrix4::rpy(0.0, -M_PI_2, 0.0) * 
                         Matrix4::rotationY(q_shoulder);

    // 3. Elbow Joint (Revolute Y)
    Matrix4 T_elbow = Matrix4::translation(0.15, 0.0, 0.0) * Matrix4::rotationY(q_elbow);

    // 4. Wrist Pitch Joint (Revolute Y)
    Matrix4 T_pitch = Matrix4::translation(0.15, 0.0, 0.0) * Matrix4::rotationY(q_pitch);

    // 5. Wrist Roll Actuator Mount (Fixed Y-offset)
    Matrix4 T_roll_actuator = Matrix4::translation(0.14, -0.072, 0.0);

    // 6. Wrist Roll Joint & Tool Mount (Revolute X)
    Matrix4 T_roll_joint = Matrix4::translation(0.02, 0.0, 0.0) * Matrix4::rotationX(q_roll);

    // Compute total transform
    Matrix4 T_total = T_rail * T_base * T_shoulder * T_elbow * T_pitch * T_roll_actuator * T_roll_joint;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "world";
    // Extract translation vector coordinates directly from matrix right column
    pose.pose.position.x = T_total.m[0][3];
    pose.pose.position.y = T_total.m[1][3];
    pose.pose.position.z = T_total.m[2][3];
    // convert 3x3 rotational matrix to quaternion
    pose.pose.orientation = matrix_to_quaternion(T_total);

    // Publish to ROS2 
    pose_pub_->publish(pose);
    // Status logging for ongoing validation and debugging
    RCLCPP_INFO(get_logger(), "FK -> X: %.4f, Y: %.4f, Z: %.4f",
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    RCLCPP_INFO(
      get_logger(), "rail %.3f base %.3f shoulder %.3f elbow %.3f wrist pitch %.3f wrist roll %.3f" ,
      q_rail, q_base, q_shoulder, q_elbow, q_pitch, q_roll);

  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

// Node initialisation entry point
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FKNode>());
  rclcpp::shutdown();
  return 0;
}