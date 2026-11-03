// Things to add (maybe): 90deg harvesting twist action, opening and closing gripper.
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
#include "std_msgs/msg/bool.hpp"
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

// Finger bodies span x=0.005 to x=0.085 from the tool mount; target their midpoint.
constexpr double graspOffset = 0.045;

// The solver aims the tool horizontally
constexpr double desiredToolPitch = kPi / 2.0;

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
  HarvestState current_state_ = HarvestState::TRACKING_TARGET;
  bool has_target_ = false; // safety latch
  std::vector<double> target_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0}; // rail, base, shoulder, elbow, pitch

  UniversalWristIKNode() : Node("ik_node")
  {
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/harvest_target_pose", 10,
      std::bind(&UniversalWristIKNode::target_callback, this, std::placeholders::_1));

    // Subscribe to current joint states to verify when we reach the target
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&UniversalWristIKNode::joint_callback, this, std::placeholders::_1));

    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/ik_node/raw_joint_targets", 10);
    harvest_complete_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/harvest/cycle_complete", 10);

    RCLCPP_INFO(get_logger(), "IK Node started with active Harvest State Machine.");
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

    // Keep the tool mount behind the berry so its center lies midway along the fingers.
    const double mountOffset = toolXOffset + graspOffset;
    const double wrist_radius = planar_radius - mountOffset * std::sin(desiredToolPitch);
    const double wrist_z = target_z - baseHeight - mountOffset * std::cos(desiredToolPitch);

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

      // URDF limits matching model
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

    // Update internal state and target cache for tracking
    target_joints_ = {rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle};
    has_target_ = true;
    current_state_ = HarvestState::TRACKING_TARGET;
    settled_feedback_count_ = 0;
    twist_settled_feedback_count_ = 0;

    send_trajectory_point(rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle);
    RCLCPP_INFO(get_logger(),
      "IK target (%.3f, %.3f, %.3f); joints (%.3f, %.3f, %.3f, %.3f, %.3f)",
      target_x, target_y, target_z, rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle);
  }

  void send_trajectory_point(
    double rail_position, double base_angle, double shoulder_angle,
    double elbow_angle, double wrist_pitch, double wrist_roll = 0.0,
    double duration_sec = 4.0)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = {
      "rail_joint", "base_joint", "shoulder_joint", "elbow_joint",
      "wrist_pitch_joint", "wrist_roll_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {rail_position, base_angle, shoulder_angle, elbow_angle, wrist_pitch, wrist_roll};
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    trajectory.points.push_back(point);
    trajectory_pub_->publish(trajectory);
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!has_target_ ||
        (current_state_ != HarvestState::TRACKING_TARGET &&
         current_state_ != HarvestState::TWISTING)) return;

    // Map joint names to their current positions
    std::unordered_map<std::string, double> current_positions;
    std::unordered_map<std::string, double> current_velocities;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->position.size()) {
        current_positions[msg->name[i]] = msg->position[i];
      }
      if (i < msg->velocity.size()) {
        current_velocities[msg->name[i]] = msg->velocity[i];
      }
    }

    // Define the joint order matching target_joints_
    std::vector<std::string> joint_names = {
      "rail_joint", "base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint"};

    // Keep the grasp locked before twisting, while allowing modest Gazebo velocity noise.
    constexpr double kPosTolerance = 0.01;
    constexpr double kVelocityTolerance = 0.15;
    constexpr int kSettledFeedbackSamples = 20;

    if (current_state_ == HarvestState::TRACKING_TARGET) {
      bool target_reached = true;
      bool missing_velocity_feedback = false;
      for (size_t i = 0; i < joint_names.size(); ++i) {
        auto current = current_positions.find(joint_names[i]);
        if (current == current_positions.end()) {
          target_reached = false;
          break;
        }
        const double error = std::abs(current->second - target_joints_[i]);
        const auto velocity = current_velocities.find(joint_names[i]);
        if (velocity == current_velocities.end()) {
          missing_velocity_feedback = true;
          target_reached = false;
          break;
        }
        if (error > kPosTolerance || std::abs(velocity->second) > kVelocityTolerance) {
          target_reached = false;
          break;
        }
      }

      if (missing_velocity_feedback) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for joint velocity feedback before declaring the target settled.");
      }

      // Require consecutive settled feedback so a single quiet sample cannot trigger a new trajectory.
      if (target_reached) {
        ++settled_feedback_count_;
        if (settled_feedback_count_ >= kSettledFeedbackSamples) {
          current_state_ = HarvestState::READY_TO_TWIST;
          RCLCPP_INFO(get_logger(), "Target Locked and Reached. Activating harvest twist sequence!");
          const auto roll_position = current_positions.find("wrist_roll_joint");
          if (roll_position == current_positions.end()) {
            RCLCPP_ERROR(get_logger(), "Cannot start harvest twist: wrist-roll feedback is missing.");
            current_state_ = HarvestState::TRACKING_TARGET;
            settled_feedback_count_ = 0;
            return;
          }
          execute_harvest_twist(roll_position->second);
        }
      } else {
        settled_feedback_count_ = 0;
      }
    } else if (current_state_ == HarvestState::TWISTING) {
      constexpr std::array<const char *, 6> all_joint_names = {
        "rail_joint", "base_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint", "wrist_roll_joint"};
      // Check the final pose with tolerances that account for simulated joint-state jitter.
      constexpr double kPositionTolerance = 0.015;
      constexpr double kRollTolerance = 0.05;
      constexpr double kVelocityTolerance = 0.15;
      constexpr int kSettledFeedbackSamples = 20;
      bool all_joints_settled = true;

      for (size_t i = 0; i < all_joint_names.size(); ++i) {
        const auto current = current_positions.find(all_joint_names[i]);
        const auto velocity = current_velocities.find(all_joint_names[i]);
        if (current == current_positions.end() || velocity == current_velocities.end()) {
          all_joints_settled = false;
          break;
        }

        const double expected_position =
          i < target_joints_.size() ? target_joints_[i] : twist_target_roll_;
        const double position_tolerance = i == target_joints_.size() ?
          kRollTolerance : kPositionTolerance;
        if (std::abs(current->second - expected_position) > position_tolerance ||
            std::abs(velocity->second) > kVelocityTolerance)
        {
          all_joints_settled = false;
          break;
        }
      }

      if (all_joints_settled) {
        ++twist_settled_feedback_count_;
      } else {
        twist_settled_feedback_count_ = 0;
      }

      if (twist_settled_feedback_count_ >= kSettledFeedbackSamples) {
        current_state_ = HarvestState::HARVEST_COMPLETE;
        std_msgs::msg::Bool completion;
        completion.data = true;
        harvest_complete_pub_->publish(completion);
        RCLCPP_INFO(
          get_logger(),
          "Harvest cycle complete: all six joints settled after the wrist-roll twist.");
      }
    }
  }

  void execute_harvest_twist(double current_roll)
  {
    // Choose a full 90-degree twist from measured roll without exceeding either joint limit.
    constexpr double kTwistAngle = kPi / 2.0;
    twist_target_roll_ = current_roll + kTwistAngle <= kPi ?
      current_roll + kTwistAngle : current_roll - kTwistAngle;

    current_state_ = HarvestState::TWISTING;

    send_trajectory_point(
      target_joints_[0], target_joints_[1], target_joints_[2],
      target_joints_[3], target_joints_[4], twist_target_roll_, 1.5);

    // Keep monitoring joint feedback; publishing a command is not proof of completion.
    RCLCPP_INFO(
      get_logger(), "Harvest twist commanded: wrist roll %.3f -> %.3f rad; waiting for feedback.",
      current_roll, twist_target_roll_);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr harvest_complete_pub_;
  int settled_feedback_count_ = 0;
  int twist_settled_feedback_count_ = 0;
  double twist_target_roll_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UniversalWristIKNode>());
  rclcpp::shutdown();
  return 0;
}