#include <algorithm>
#include <array>
#include <chrono>
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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

// Updated kinematics based on URDF offsets
constexpr double railMin = -0.9;
constexpr double railMax = 0.9;

// Include Gazebo's 25 mm model spawn height and the URDF rail/base/shoulder offsets.
constexpr double baseHeight = 0.025 + 0.025 + 0.108 + 0.061;
constexpr double upperArmLength = 0.15;
constexpr double forearmLength = 0.15;

// Tool X offset: 0.14 (actuator mount) + 0.02 (tool mount) = 0.16 m
// Tool Y offset: -0.072 (actuator mount introduces a non-planar shift)
constexpr double toolXOffset = 0.16;
constexpr double toolYOffset = -0.072;

// Finger bodies span x=0.005 to x=0.085 from the tool mount; target their midpoint.
constexpr double graspOffset = 0.045;

// Prefer a horizontal tool, but allow a small tilt when needed to satisfy joint limits.
constexpr double desiredToolPitch = kPi / 2.0;
constexpr double kToolPitchSearchStep = 0.01;
constexpr int kToolPitchSearchSteps = 50;
constexpr double kJointLimitMargin = 0.03;
constexpr std::array<double, 6> kSettlePositionTolerance = {
  0.05, 0.2, 0.2, 0.2, 0.2, 0.2};
constexpr double kGraspPositionTolerance = 0.05;
constexpr double kSettleVelocityTolerance = 0.03;
constexpr double kSettleDurationSec = 0.20;
constexpr std::chrono::seconds kFingerActionTimeout{4};
// The shallow closure works with the URDF's low effort cap to avoid squeezing the berry.
constexpr double kClosedFingerPosition = -0.17;
constexpr double kOpenFingerPosition = 0.0;
constexpr double kFingerPositionTolerance = 0.02;
constexpr std::array<const char *, 3> kFingerJointNames = {
  "finger_joint_1", "finger_joint_2", "finger_joint_3"};

bool inside_range(double value, double lower, double upper)
{
  return value >= lower && value <= upper;
}

std::array<double, 3> grasp_point_from_joints(
  const std::array<double, 6> & joints)
{
  const double rail = joints[0];
  const double base = joints[1];
  const double shoulder = joints[2];
  const double elbow = joints[3];
  const double wrist_pitch = joints[4];
  const double tool_pitch = shoulder + elbow + wrist_pitch;
  const double radial_position =
    upperArmLength * std::sin(shoulder) +
    forearmLength * std::sin(shoulder + elbow) +
    (toolXOffset + graspOffset) * std::sin(tool_pitch);

  // Match FK's URDF chain, including the lateral wrist actuator offset and finger midpoint.
  return {
    rail + radial_position * std::cos(base) - toolYOffset * std::sin(base),
    radial_position * std::sin(base) + toolYOffset * std::cos(base),
    baseHeight +
    upperArmLength * std::cos(shoulder) +
    forearmLength * std::cos(shoulder + elbow) +
    (toolXOffset + graspOffset) * std::cos(tool_pitch)};
}
}  // namespace

// enum definition for the harvest state machine
enum class HarvestState {
  TRACKING_TARGET,
  CLOSING_GRIPPER,
  TWISTING,
  OPENING_GRIPPER,
  HARVEST_COMPLETE,
  HARVEST_FAILED
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
    gripper_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_controller/commands", 10);

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
    const double mount_offset = toolXOffset + graspOffset;
    bool solution_found = false;
    double shoulder_angle = 0.0;
    double elbow_angle = 0.0;
    double wrist_angle = 0.0;
    double selected_tool_pitch = desiredToolPitch;

    // Search outward from the preferred tool angle and choose the smallest feasible tilt.
    for (int pitch_step = 0; pitch_step <= kToolPitchSearchSteps && !solution_found; ++pitch_step) {
      for (const double direction : std::array<double, 2>{1.0, -1.0}) {
        if (pitch_step == 0 && direction < 0.0) {
          continue;
        }
        const double try_tool_pitch =
          desiredToolPitch + direction * pitch_step * kToolPitchSearchStep;
        const double wrist_radius = planar_radius - mount_offset * std::sin(try_tool_pitch);
        const double wrist_z =
          target_z - baseHeight - mount_offset * std::cos(try_tool_pitch);
        const double reach_squared = wrist_radius * wrist_radius + wrist_z * wrist_z;
        double cos_elbow =
          (reach_squared - upperArmLength * upperArmLength - forearmLength * forearmLength) /
          (2.0 * upperArmLength * forearmLength);
        if (cos_elbow < -1.0 - 1e-9 || cos_elbow > 1.0 + 1e-9) {
          continue;
        }
        cos_elbow = std::clamp(cos_elbow, -1.0, 1.0);

        for (const double elbow_sign : std::array<double, 2>{1.0, -1.0}) {
          const double try_elbow = elbow_sign * std::acos(cos_elbow);
          const double try_shoulder =
            std::atan2(wrist_radius, wrist_z) -
            std::atan2(forearmLength * std::sin(try_elbow),
                       upperArmLength + forearmLength * std::cos(try_elbow));
          const double try_wrist = try_tool_pitch - try_shoulder - try_elbow;

          // Leave a small margin so the arm does not settle against a hard joint stop.
          if (inside_range(try_shoulder, -1.5708 + kJointLimitMargin, 1.5708 - kJointLimitMargin) &&
              inside_range(try_elbow, -2.0 + kJointLimitMargin, 2.0 - kJointLimitMargin) &&
              inside_range(try_wrist, -1.5708 + kJointLimitMargin, 1.5708 - kJointLimitMargin))
          {
            shoulder_angle = try_shoulder;
            elbow_angle = try_elbow;
            wrist_angle = try_wrist;
            selected_tool_pitch = try_tool_pitch;
            solution_found = true;
            break;
          }
        }
        if (solution_found) {
          break;
        }
      }
    }

    if (!solution_found) {
      RCLCPP_ERROR(
        get_logger(),
        "Target (%.3f, %.3f, %.3f) is unreachable within joint limits and a +/- %.2f rad "
        "tool-pitch adjustment.",
        target_x, target_y, target_z, kToolPitchSearchSteps * kToolPitchSearchStep);
      return;
    }

    // Update internal state and target cache for tracking
    target_joints_ = {rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle};
    target_x_ = target_x;
    target_y_ = target_y;
    target_z_ = target_z;
    has_target_ = true;
    current_state_ = HarvestState::TRACKING_TARGET;
    target_settled_since_valid_ = false;
    twist_settled_since_valid_ = false;

    send_trajectory_point(rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle);
    RCLCPP_INFO(get_logger(),
      "IK target (%.3f, %.3f, %.3f); joints (%.3f, %.3f, %.3f, %.3f, %.3f)",
      target_x, target_y, target_z, rail_position, base_angle, shoulder_angle, elbow_angle, wrist_angle);
    if (std::abs(selected_tool_pitch - desiredToolPitch) > 1e-6) {
      RCLCPP_INFO(
        get_logger(), "Adjusted tool pitch from %.3f to %.3f rad to keep the grasp reachable.",
        desiredToolPitch, selected_tool_pitch);
    }
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

  void send_wrist_roll_trajectory_point(double wrist_roll, double duration_sec)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = {"wrist_roll_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {wrist_roll};
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    trajectory.points.push_back(point);
    trajectory_pub_->publish(trajectory);
  }

  // Command all three hinges together so the berry is held evenly during the twist.
  void command_gripper(double position)
  {
    std_msgs::msg::Float64MultiArray command;
    command.data.assign(kFingerJointNames.size(), position);
    gripper_command_pub_->publish(command);
  }

  bool gripper_reached_target(
    const std::unordered_map<std::string, double> & current_positions,
    const std::unordered_map<std::string, double> & current_velocities,
    double target_position)
  {
    for (const char * joint_name : kFingerJointNames) {
      const auto position = current_positions.find(joint_name);
      const auto velocity = current_velocities.find(joint_name);
      if (position == current_positions.end() || velocity == current_velocities.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for position and velocity feedback for gripper joint '%s'.", joint_name);
        return false;
      }
      if (std::abs(position->second - target_position) > kFingerPositionTolerance ||
          std::abs(velocity->second) > kSettleVelocityTolerance)
      {
        RCLCPP_DEBUG(
          get_logger(), "Gripper joint %s is still moving or outside its target tolerance.",
          joint_name);
        return false;
      }
    }
    return true;
  }

  bool gripper_is_stationary(
    const std::unordered_map<std::string, double> & current_positions,
    const std::unordered_map<std::string, double> & current_velocities)
  {
    for (const char * joint_name : kFingerJointNames) {
      if (current_positions.find(joint_name) == current_positions.end()) {
        return false;
      }
      const auto velocity = current_velocities.find(joint_name);
      if (velocity == current_velocities.end() ||
          std::abs(velocity->second) > kSettleVelocityTolerance)
      {
        return false;
      }
    }
    return true;
  }

  void fail_harvest(const char * reason)
  {
    current_state_ = HarvestState::HARVEST_FAILED;
    std_msgs::msg::Bool completion;
    completion.data = false;
    harvest_complete_pub_->publish(completion);
    RCLCPP_ERROR(get_logger(), "Harvest cycle failed: %s", reason);
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!has_target_ ||
        (current_state_ != HarvestState::TRACKING_TARGET &&
         current_state_ != HarvestState::CLOSING_GRIPPER &&
         current_state_ != HarvestState::TWISTING &&
         current_state_ != HarvestState::OPENING_GRIPPER)) return;

    const rclcpp::Time feedback_time = now();

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

    // Include wrist roll's commanded home position so no axis is still moving at the target.
    constexpr std::array<const char *, 6> joint_names = {
      "rail_joint", "base_joint", "shoulder_joint", "elbow_joint",
      "wrist_pitch_joint", "wrist_roll_joint"};

    if (current_state_ == HarvestState::TRACKING_TARGET) {
      bool target_reached = true;
      bool missing_joint_feedback = false;
      double max_position_error = 0.0;
      double max_joint_velocity = 0.0;
      std::array<double, 6> measured_joint_positions{};
      const char * limiting_joint = "none";
      for (size_t i = 0; i < joint_names.size(); ++i) {
        auto current = current_positions.find(joint_names[i]);
        if (current == current_positions.end()) {
          target_reached = false;
          missing_joint_feedback = true;
          limiting_joint = joint_names[i];
          break;
        }
        measured_joint_positions[i] = current->second;
        const double expected_position = i < target_joints_.size() ? target_joints_[i] : 0.0;
        double error = std::abs(current->second - expected_position);
        if (i > 0) {
          error = std::abs(std::remainder(current->second - expected_position, 2.0 * kPi));
        }
        max_position_error = std::max(max_position_error, error);
        bool joint_settled = error <= kSettlePositionTolerance[i];
        const auto velocity = current_velocities.find(joint_names[i]);
        if (velocity == current_velocities.end()) {
          missing_joint_feedback = true;
          target_reached = false;
          limiting_joint = joint_names[i];
          break;
        }
        const double joint_velocity = std::abs(velocity->second);
        max_joint_velocity = std::max(max_joint_velocity, joint_velocity);
        if (joint_velocity > kSettleVelocityTolerance) {
          joint_settled = false;
        }
        if (!joint_settled) {
          target_reached = false;
          limiting_joint = joint_names[i];
          RCLCPP_DEBUG(
            get_logger(), "Joint %s not settled: error %.4f, speed %.4f.",
            joint_names[i], error, joint_velocity);
        }
      }

      double grasp_position_error = 0.0;
      if (!missing_joint_feedback) {
        const auto grasp_point = grasp_point_from_joints(measured_joint_positions);
        const double dx = grasp_point[0] - target_x_;
        const double dy = grasp_point[1] - target_y_;
        const double dz = grasp_point[2] - target_z_;
        grasp_position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (grasp_position_error > kGraspPositionTolerance) {
          target_reached = false;
          limiting_joint = "grasp_point";
        }
      }

      if (missing_joint_feedback) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for position and velocity feedback for joint '%s'.", limiting_joint);
      }

      if (!target_reached) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting to harvest: limiting joint %s; max position error %.4f, "
          "grasp-point error %.4f m, max joint speed %.4f "
          "(rail tolerance %.3f m, rotary tolerance %.3f rad, "
          "grasp tolerance %.3f m, speed tolerance %.3f).",
          limiting_joint, max_position_error, grasp_position_error, max_joint_velocity,
          kSettlePositionTolerance[0], kSettlePositionTolerance[1],
          kGraspPositionTolerance, kSettleVelocityTolerance);
      }

      if (target_reached) {
        if (!target_settled_since_valid_) {
          target_settled_since_ = feedback_time;
          target_settled_since_valid_ = true;
        } else if ((feedback_time - target_settled_since_).seconds() >= kSettleDurationSec) {
          // Start the twist only after the fingers have reached their gentle grasp position.
          current_state_ = HarvestState::CLOSING_GRIPPER;
          gripper_settled_since_valid_ = false;
          finger_action_started_ = std::chrono::steady_clock::now();
          command_gripper(kClosedFingerPosition);
          RCLCPP_INFO(
            get_logger(), "Berry target is settled; gently closing the gripper before twisting.");
        }
      } else {
        target_settled_since_valid_ = false;
      }
    } else if (current_state_ == HarvestState::CLOSING_GRIPPER) {
      const auto steady_now = std::chrono::steady_clock::now();
      const bool close_timed_out =
        steady_now - finger_action_started_ >= kFingerActionTimeout;
      if (gripper_reached_target(
          current_positions, current_velocities, kClosedFingerPosition))
      {
        if (!gripper_settled_since_valid_) {
          gripper_settled_since_ = steady_now;
          gripper_settled_since_valid_ = true;
        }
      } else {
        gripper_settled_since_valid_ = false;
      }

      const bool fingers_settled_at_target =
        gripper_settled_since_valid_ &&
        std::chrono::duration<double>(steady_now - gripper_settled_since_).count() >=
        kSettleDurationSec;
      const bool stopped_at_contact =
        close_timed_out && gripper_is_stationary(current_positions, current_velocities);

      if (close_timed_out && !fingers_settled_at_target && !stopped_at_contact) {
        fail_harvest(
          "gripper did not reach its target or stop safely before the close timeout.");
        return;
      }

      if (fingers_settled_at_target || stopped_at_contact) {
        if (stopped_at_contact && !fingers_settled_at_target) {
          // A berry can stop the low-effort fingers before they reach the no-contact target.
          RCLCPP_WARN(
            get_logger(), "Gripper stopped at berry contact; continuing with the harvest twist.");
        }
        const auto roll_position = current_positions.find("wrist_roll_joint");
        if (roll_position == current_positions.end()) {
          if (close_timed_out) {
            fail_harvest("wrist-roll feedback was unavailable after the gripper close timeout.");
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Cannot start harvest twist until wrist-roll feedback is available.");
          }
          return;
        }
        execute_harvest_twist(roll_position->second);
      }
    } else if (current_state_ == HarvestState::TWISTING) {
      constexpr std::array<const char *, 6> all_joint_names = {
        "rail_joint", "base_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint", "wrist_roll_joint"};
      // Check the final pose with tolerances that account for simulated joint-state jitter.
      bool all_joints_settled = true;
      double max_position_error = 0.0;
      double max_joint_velocity = 0.0;
      std::array<double, 6> measured_joint_positions{};
      bool missing_joint_feedback = false;
      const char * limiting_joint = "none";

      for (size_t i = 0; i < all_joint_names.size(); ++i) {
        const auto current = current_positions.find(all_joint_names[i]);
        const auto velocity = current_velocities.find(all_joint_names[i]);
        if (current == current_positions.end() || velocity == current_velocities.end()) {
          all_joints_settled = false;
          missing_joint_feedback = true;
          limiting_joint = all_joint_names[i];
          break;
        }
        measured_joint_positions[i] = current->second;

        const double expected_position =
          i < target_joints_.size() ? target_joints_[i] : twist_target_roll_;
        double error = std::abs(current->second - expected_position);
        if (i > 0) {
          error = std::abs(std::remainder(current->second - expected_position, 2.0 * kPi));
        }
        const double joint_velocity = std::abs(velocity->second);
        max_position_error = std::max(max_position_error, error);
        max_joint_velocity = std::max(max_joint_velocity, joint_velocity);
        if (error > kSettlePositionTolerance[i] ||
            joint_velocity > kSettleVelocityTolerance)
        {
          all_joints_settled = false;
          limiting_joint = all_joint_names[i];
        }
      }

      double grasp_position_error = 0.0;
      if (!missing_joint_feedback) {
        const auto grasp_point = grasp_point_from_joints(measured_joint_positions);
        const double dx = grasp_point[0] - target_x_;
        const double dy = grasp_point[1] - target_y_;
        const double dz = grasp_point[2] - target_z_;
        grasp_position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (grasp_position_error > kGraspPositionTolerance) {
          all_joints_settled = false;
          limiting_joint = "grasp_point";
        }
      }

      if (all_joints_settled) {
        if (!twist_settled_since_valid_) {
          twist_settled_since_ = feedback_time;
          twist_settled_since_valid_ = true;
        }
      } else {
        twist_settled_since_valid_ = false;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for twist to settle: limiting joint %s; max position error %.4f, "
          "grasp-point error %.4f m, max joint speed %.4f.",
          limiting_joint, max_position_error, grasp_position_error, max_joint_velocity);
      }

      if (twist_settled_since_valid_ &&
          (feedback_time - twist_settled_since_).seconds() >= kSettleDurationSec)
      {
        // Release only after the wrist twist is confirmed settled; report completion after opening.
        current_state_ = HarvestState::OPENING_GRIPPER;
        gripper_settled_since_valid_ = false;
        finger_action_started_ = std::chrono::steady_clock::now();
        command_gripper(kOpenFingerPosition);
        RCLCPP_INFO(get_logger(), "Wrist twist settled; opening the gripper to finish harvesting.");
      }
    } else if (current_state_ == HarvestState::OPENING_GRIPPER) {
      const auto steady_now = std::chrono::steady_clock::now();
      if (gripper_reached_target(current_positions, current_velocities, kOpenFingerPosition)) {
        if (!gripper_settled_since_valid_) {
          gripper_settled_since_ = steady_now;
          gripper_settled_since_valid_ = true;
        }
      } else {
        gripper_settled_since_valid_ = false;
      }

      const bool fingers_open_and_settled =
        gripper_settled_since_valid_ &&
        std::chrono::duration<double>(steady_now - gripper_settled_since_).count() >=
        kSettleDurationSec;
      if (fingers_open_and_settled) {
        current_state_ = HarvestState::HARVEST_COMPLETE;
        std_msgs::msg::Bool completion;
        completion.data = true;
        harvest_complete_pub_->publish(completion);
        RCLCPP_INFO(
          get_logger(),
          "Harvest cycle complete: the wrist twist settled and all gripper fingers reopened.");
      } else if (steady_now - finger_action_started_ >= kFingerActionTimeout) {
        fail_harvest("gripper did not reopen before the open timeout.");
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
    twist_settled_since_valid_ = false;

    // Let the planner hold the other five joints at their measured positions during the twist.
    send_wrist_roll_trajectory_point(twist_target_roll_, 1.5);

    // Keep monitoring joint feedback; publishing a command is not proof of completion.
    RCLCPP_INFO(
      get_logger(), "Harvest twist commanded: wrist roll %.3f -> %.3f rad; waiting for feedback.",
      current_roll, twist_target_roll_);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr harvest_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_pub_;
  rclcpp::Time target_settled_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time twist_settled_since_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point finger_action_started_;
  std::chrono::steady_clock::time_point gripper_settled_since_;
  bool target_settled_since_valid_ = false;
  bool twist_settled_since_valid_ = false;
  bool gripper_settled_since_valid_ = false;
  double twist_target_roll_ = 0.0;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_z_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UniversalWristIKNode>());
  rclcpp::shutdown();
  return 0;
}