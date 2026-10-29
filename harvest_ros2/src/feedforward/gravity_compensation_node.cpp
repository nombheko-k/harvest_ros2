#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class CompletePIDGravityController : public rclcpp::Node
{
public:
    CompletePIDGravityController() : Node("position_plus_gravity_controller")
    {
        // 1. Declare Parameters matching your updated manual tuning profile
        this->declare_parameter<double>("rail.kp", 1200.0);   this->declare_parameter<double>("rail.ki", 5.0);   this->declare_parameter<double>("rail.kd", 50.0);   this->declare_parameter<double>("rail.i_clamp", 10.0);  this->declare_parameter<double>("rail.limit", 200.0); // Updated to 200.0 Nm physical constraint
        this->declare_parameter<double>("base.kp", 80.0);     this->declare_parameter<double>("base.ki", 0.13);  this->declare_parameter<double>("base.kd", 5.33);   this->declare_parameter<double>("base.i_clamp", 1.0);   this->declare_parameter<double>("base.limit", 4.0);
        
        // Shoulder
        this->declare_parameter<double>("shoulder.kp", 200.0); 
        this->declare_parameter<double>("shoulder.ki", 2.67); 
        this->declare_parameter<double>("shoulder.kd", 10.67); 
        this->declare_parameter<double>("shoulder.i_clamp", 4.5); 
        this->declare_parameter<double>("shoulder.limit", 5.60);
        this->declare_parameter<double>("shoulder.max_gravity", 3.92); // Slightly scaled to lift the extra elbow weight

        // Elbow
        this->declare_parameter<double>("elbow.kp", 133.33);  this->declare_parameter<double>("elbow.ki", 1.33); this->declare_parameter<double>("elbow.kd", 8.0);    this->declare_parameter<double>("elbow.i_clamp", 3.5);  this->declare_parameter<double>("elbow.limit", 4.0);
        this->declare_parameter<double>("wrist_p.kp", 33.33); this->declare_parameter<double>("wrist_p.ki", 0.67);this->declare_parameter<double>("wrist_p.kd", 2.0);   this->declare_parameter<double>("wrist_p.i_clamp", 1.0);this->declare_parameter<double>("wrist_p.limit", 4.0);
        this->declare_parameter<double>("wrist_r.kp", 6.67);  this->declare_parameter<double>("wrist_r.ki", 0.01);this->declare_parameter<double>("wrist_r.kd", 0.27);  this->declare_parameter<double>("wrist_r.i_clamp", 0.5);this->declare_parameter<double>("wrist_r.limit", 4.0);

        joint_names_ = {"rail_joint", "base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"};
        
        current_positions_.resize(joint_names_.size(), 0.0);
        current_velocities_.resize(joint_names_.size(), 0.0);
        target_positions_.resize(joint_names_.size(), 0.0);
        target_velocities_.resize(joint_names_.size(), 0.0);
        
        // Error history initialization structures to run native Integral math functions
        integral_errors_.resize(joint_names_.size(), 0.0);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&CompletePIDGravityController::jointStateCallback, this, std::placeholders::_1));

        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10, std::bind(&CompletePIDGravityController::trajectoryCallback, this, std::placeholders::_1));

        effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&CompletePIDGravityController::controlLoop, this));

        trajectory_active_ = false;
        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Gravity Compensation Node initialised.");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < joint_names_.size(); ++j) {
                if (msg->name[i] == joint_names_[j]) {
                    current_positions_[j] = msg->position[i];
                    current_velocities_[j] = msg->velocity[i];
                }
            }
        }
    }

    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty()) return;
        active_trajectory_ = *msg;
        trajectory_start_time_ = this->now();
        trajectory_active_ = true;
    }

    void controlLoop()
    {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0) return;
        last_time_ = now;

        if (trajectory_active_) {
            double elapsed_time = (now - trajectory_start_time_).seconds();
            size_t point_idx = 0;
            bool found = false;
            
            for (size_t i = 0; i < active_trajectory_.points.size(); ++i) {
                double point_time = rclcpp::Duration(active_trajectory_.points[i].time_from_start).seconds();
                if (point_time >= elapsed_time) {
                    point_idx = i;
                    found = true;
                    break;
                }
            }

            if (!found && !active_trajectory_.points.empty()) {
                point_idx = active_trajectory_.points.size() - 1;
            }

            trajectory_msgs::msg::JointTrajectoryPoint active_point = active_trajectory_.points[point_idx];

            for (size_t j = 0; j < joint_names_.size(); ++j) {
                auto it = std::find(active_trajectory_.joint_names.begin(), active_trajectory_.joint_names.end(), joint_names_[j]);
                if (it != active_trajectory_.joint_names.end()) {
                    size_t idx = std::distance(active_trajectory_.joint_names.begin(), it);
                    if (idx < active_point.positions.size()) target_positions_[j] = active_point.positions[idx];
                    if (idx < active_point.velocities.size()) target_velocities_[j] = active_point.velocities[idx];
                }
            }
        }

        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.resize(joint_names_.size(), 0.0);

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            double pos_error = target_positions_[i] - current_positions_[i];
            double vel_error = target_velocities_[i] - current_velocities_[i];

            double kp = 0.0, ki = 0.0, kd = 0.0, i_clamp = 0.0, limit = 4.0;
            double ff_gravity = 0.0;

            // Load joint configs dynamically
            if (joint_names_[i] == "rail_joint") {
                kp = this->get_parameter("rail.kp").as_double();     ki = this->get_parameter("rail.ki").as_double();     kd = this->get_parameter("rail.kd").as_double();
                i_clamp = this->get_parameter("rail.i_clamp").as_double(); limit = this->get_parameter("rail.limit").as_double();
            } else if (joint_names_[i] == "base_joint") {
                kp = this->get_parameter("base.kp").as_double();     ki = this->get_parameter("base.ki").as_double();     kd = this->get_parameter("base.kd").as_double();
                i_clamp = this->get_parameter("base.i_clamp").as_double();  limit = this->get_parameter("base.limit").as_double();
            } else if (joint_names_[i] == "shoulder_joint") {
                kp = this->get_parameter("shoulder.kp").as_double(); ki = this->get_parameter("shoulder.ki").as_double(); kd = this->get_parameter("shoulder.kd").as_double();
                i_clamp = this->get_parameter("shoulder.i_clamp").as_double(); limit = this->get_parameter("shoulder.limit").as_double();
                
                double max_grav = this->get_parameter("shoulder.max_gravity").as_double();
                ff_gravity = max_grav * std::sin(current_positions_[i]);
            } else if (joint_names_[i] == "elbow_joint") {
                kp = this->get_parameter("elbow.kp").as_double();    ki = this->get_parameter("elbow.ki").as_double();    kd = this->get_parameter("elbow.kd").as_double();
                i_clamp = this->get_parameter("elbow.i_clamp").as_double(); limit = this->get_parameter("elbow.limit").as_double();
            } else if (joint_names_[i] == "wrist_pitch_joint") {
                kp = this->get_parameter("wrist_p.kp").as_double();  ki = this->get_parameter("wrist_p.ki").as_double();  kd = this->get_parameter("wrist_p.kd").as_double();
                i_clamp = this->get_parameter("wrist_p.i_clamp").as_double(); limit = this->get_parameter("wrist_p.limit").as_double();
            } else if (joint_names_[i] == "wrist_roll_joint") {
                kp = this->get_parameter("wrist_r.kp").as_double();  ki = this->get_parameter("wrist_r.ki").as_double();  kd = this->get_parameter("wrist_r.kd").as_double();
                i_clamp = this->get_parameter("wrist_r.i_clamp").as_double(); limit = this->get_parameter("wrist_r.limit").as_double();
            }

            // NATIVE ROS2 INTEGRAL ERRORS WINDUP PROTECTION HANDLING 
            integral_errors_[i] += pos_error * dt;
            integral_errors_[i] = std::clamp(integral_errors_[i], -i_clamp, i_clamp);

            // Complete Parallel PID Execution Equation + Feedforward 
            double raw_effort = (kp * pos_error) + (ki * integral_errors_[i]) + (kd * vel_error) + ff_gravity;
            effort_msg.data[i] = std::clamp(raw_effort, -limit, limit);
        }

        effort_pub_->publish(effort_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    std::vector<double> current_positions_;
    std::vector<double> current_velocities_;
    std::vector<double> target_positions_;
    std::vector<double> target_velocities_;
    std::vector<double> integral_errors_;
    trajectory_msgs::msg::JointTrajectory active_trajectory_;
    rclcpp::Time trajectory_start_time_;
    rclcpp::Time last_time_;
    bool trajectory_active_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Add the class type inside the make_shared template angle brackets
    rclcpp::spin(std::make_shared<CompletePIDGravityController>());
    rclcpp::shutdown();
    return 0;
}
