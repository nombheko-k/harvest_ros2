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
        // Use lower position gains and more velocity damping to reduce oscillation;
        // small integral bounds compensate residual bias without winding up at the target.
        // Rail
        this->declare_parameter<double>("rail.kp", 350.0);
        this->declare_parameter<double>("rail.ki", 2.0);
        this->declare_parameter<double>("rail.kd", 120.0);
        this->declare_parameter<double>("rail.i_clamp", 1.5);
        this->declare_parameter<double>("rail.limit", 200.0); // Prismatic effort limit is in newtons.

        // Base
        this->declare_parameter<double>("base.kp", 40.0);
        this->declare_parameter<double>("base.ki", 0.2);
        this->declare_parameter<double>("base.kd", 4.0);
        this->declare_parameter<double>("base.i_clamp", 1.0);
        this->declare_parameter<double>("base.limit", 4.0);

        // Shoulder
        this->declare_parameter<double>("shoulder.kp", 60.0);
        this->declare_parameter<double>("shoulder.ki", 0.5);
        this->declare_parameter<double>("shoulder.kd", 6.0);
        this->declare_parameter<double>("shoulder.i_clamp", 1.0);
        this->declare_parameter<double>("shoulder.limit", 30.0);
        this->declare_parameter<double>("shoulder.max_gravity", 6.95);

        // Elbow
        this->declare_parameter<double>("elbow.kp", 45.0);
        this->declare_parameter<double>("elbow.ki", 0.2);
        this->declare_parameter<double>("elbow.kd", 4.5);
        this->declare_parameter<double>("elbow.i_clamp", 0.5);
        this->declare_parameter<double>("elbow.limit", 15.0);
        this->declare_parameter<double>("elbow.max_gravity", 3.80);
        
        // Wrist pitch
        this->declare_parameter<double>("wrist_p.kp", 30.0);
        this->declare_parameter<double>("wrist_p.ki", 0.0);
        this->declare_parameter<double>("wrist_p.kd", 3.0);
        this->declare_parameter<double>("wrist_p.i_clamp", 0.0);
        this->declare_parameter<double>("wrist_p.limit", 10.0);
        this->declare_parameter<double>("wrist_p.max_gravity", 1.63);

        // wrist roll
        this->declare_parameter<double>("wrist_r.kp", 8.0);
        this->declare_parameter<double>("wrist_r.ki", 0.1);
        this->declare_parameter<double>("wrist_r.kd", 0.8);
        this->declare_parameter<double>("wrist_r.i_clamp", 1.0);
        this->declare_parameter<double>("wrist_r.limit", 4.0);

        joint_names_ = {"rail_joint", "base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"};
        
        current_positions_.resize(joint_names_.size(), 0.0);
        current_velocities_.resize(joint_names_.size(), 0.0);
        target_positions_.resize(joint_names_.size(), 0.0);
        target_velocities_.resize(joint_names_.size(), 0.0);
        smoothed_vel_errors_.resize(joint_names_.size(), 0.0);
        joint_state_seen_.resize(joint_names_.size(), false);
        
        // Error history initialization structures to run native Integral math functions
        integral_errors_.resize(joint_names_.size(), 0.0);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&CompletePIDGravityController::jointStateCallback, this, std::placeholders::_1));

        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10, std::bind(&CompletePIDGravityController::trajectoryCallback, this, std::placeholders::_1));

        effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);

        // Change to 200Hz 
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&CompletePIDGravityController::controlLoop, this));

        trajectory_active_ = false;
        target_initialized_ = false;
        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Gravity Compensation Node initialised.");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < joint_names_.size(); ++j) {
                if (msg->name[i] == joint_names_[j] && i < msg->position.size()) {
                    current_positions_[j] = msg->position[i];
                    // JointState may omit velocities; avoid indexing beyond its available feedback.
                    current_velocities_[j] =
                        i < msg->velocity.size() ? msg->velocity[i] : 0.0;
                    joint_state_seen_[j] = true;
                    if (!target_initialized_) {
                        // Seed the hold reference from measured state before the first commanded trajectory.
                        target_positions_[j] = current_positions_[j];
                    }
                }
            }
        }

        if (!target_initialized_ &&
            std::all_of(joint_state_seen_.begin(), joint_state_seen_.end(), [](bool seen) { return seen; }))
        {
            target_initialized_ = true;
        }
    }

    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty()) return;
        active_trajectory_ = *msg;
        trajectory_start_time_ = this->now();
        trajectory_active_ = true;
        target_initialized_ = true;
        settled_counter_ = 0;
        // Keep learned static-load correction across moves; only clear velocity-filter memory.
        std::fill(smoothed_vel_errors_.begin(), smoothed_vel_errors_.end(), 0.0);
    }

    void controlLoop()
    {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0) return;
        last_time_ = now;
        // Do not command joints until feedback has seeded every hold target.
        if (!target_initialized_) return;

        if (trajectory_active_) {
            double elapsed_time = (now - trajectory_start_time_).seconds();
            size_t lower_idx = 0;
            size_t upper_idx = 0;
            double interpolation = 0.0;

            const auto & points = active_trajectory_.points;
            if (elapsed_time >=
                rclcpp::Duration(points.back().time_from_start).seconds())
            {
                lower_idx = upper_idx = points.size() - 1;

                // Ensure final trajectory point is held exactly
                for (size_t j = 0; j < joint_names_.size(); ++j)
                {
                    auto it = std::find(
                        active_trajectory_.joint_names.begin(),
                        active_trajectory_.joint_names.end(),
                        joint_names_[j]);

                    if (it != active_trajectory_.joint_names.end())
                    {
                        size_t idx =
                            std::distance(active_trajectory_.joint_names.begin(), it);

                        if (idx < points.back().positions.size())
                        {
                            target_positions_[j] =
                                points.back().positions[idx];
                        }

                        target_velocities_[j] = 0.0;
                    }
                }
            } else if (elapsed_time > rclcpp::Duration(points.front().time_from_start).seconds()) {
                for (size_t i = 1; i < points.size(); ++i) {
                    const double upper_time = rclcpp::Duration(points[i].time_from_start).seconds();
                    if (upper_time >= elapsed_time) {
                        lower_idx = i - 1;
                        upper_idx = i;
                        const double lower_time =
                            rclcpp::Duration(points[lower_idx].time_from_start).seconds();
                        const double interval = upper_time - lower_time;
                        interpolation = interval > 0.0 ? (elapsed_time - lower_time) / interval : 1.0;
                        break;
                    }
                }
            }

            for (size_t j = 0; j < joint_names_.size(); ++j) {
                auto it = std::find(active_trajectory_.joint_names.begin(), active_trajectory_.joint_names.end(), joint_names_[j]);
                if (it != active_trajectory_.joint_names.end()) {
                    size_t idx = std::distance(active_trajectory_.joint_names.begin(), it);
                    const auto & lower_point = points[lower_idx];
                    const auto & upper_point = points[upper_idx];
                    if (idx < lower_point.positions.size() && idx < upper_point.positions.size()) {
                        // Interpolate between neighboring planner samples to avoid staircase tracking error.
                        target_positions_[j] = lower_point.positions[idx] +
                            interpolation * (upper_point.positions[idx] - lower_point.positions[idx]);
                    }
                    if (idx < lower_point.velocities.size() && idx < upper_point.velocities.size()) {
                        // Keep velocity feed-forward synchronized with the interpolated position reference.
                        target_velocities_[j] = lower_point.velocities[idx] +
                            interpolation * (upper_point.velocities[idx] - lower_point.velocities[idx]);
                    } else {
                        target_velocities_[j] = 0.0;
                    }
                }
            }
        }

        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.resize(joint_names_.size(), 0.0);

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            double pos_error = target_positions_[i] - current_positions_[i];
            double raw_vel_error = target_velocities_[i] - current_velocities_[i];
            
            smoothed_vel_errors_[i] = (0.8 * smoothed_vel_errors_[i]) + (0.2 * raw_vel_error);
            
            double vel_error = smoothed_vel_errors_[i];

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
                // This coefficient sums the downstream URDF masses' lever arms at the shoulder.
                double eval_position = target_initialized_ ? target_positions_[i] : current_positions_[i];
                ff_gravity = -max_grav * std::sin(eval_position);
            } else if (joint_names_[i] == "elbow_joint") {
                kp = this->get_parameter("elbow.kp").as_double();    ki = this->get_parameter("elbow.ki").as_double();    kd = this->get_parameter("elbow.kd").as_double();
                i_clamp = this->get_parameter("elbow.i_clamp").as_double(); limit = this->get_parameter("elbow.limit").as_double();
                // Elbow gravity depends on the absolute arm pitch, not elbow angle alone.
                double max_grav_el = this->get_parameter("elbow.max_gravity").as_double();
                const double shoulder_angle = target_initialized_ ? target_positions_[2] : current_positions_[2];
                const double elbow_angle = target_initialized_ ? target_positions_[3] : current_positions_[3];
                ff_gravity = -max_grav_el * std::sin(shoulder_angle + elbow_angle);
            } else if (joint_names_[i] == "wrist_pitch_joint") {
                kp = this->get_parameter("wrist_p.kp").as_double();  ki = this->get_parameter("wrist_p.ki").as_double();  kd = this->get_parameter("wrist_p.kd").as_double();
                i_clamp = this->get_parameter("wrist_p.i_clamp").as_double(); limit = this->get_parameter("wrist_p.limit").as_double();
                // Wrist-pitch gravity uses the cumulative pitch of all three arm links.
                double max_grav_wp = this->get_parameter("wrist_p.max_gravity").as_double();
                const double shoulder_angle = target_initialized_ ? target_positions_[2] : current_positions_[2];
                const double elbow_angle = target_initialized_ ? target_positions_[3] : current_positions_[3];
                const double wrist_pitch = target_initialized_ ? target_positions_[4] : current_positions_[4];
                ff_gravity = -max_grav_wp * std::sin(shoulder_angle + elbow_angle + wrist_pitch);
            } else if (joint_names_[i] == "wrist_roll_joint") {
                kp = this->get_parameter("wrist_r.kp").as_double();  ki = this->get_parameter("wrist_r.ki").as_double();  kd = this->get_parameter("wrist_r.kd").as_double();
                i_clamp = this->get_parameter("wrist_r.i_clamp").as_double(); limit = this->get_parameter("wrist_r.limit").as_double();
            }

            // Accumulate small steady-state arm errors so gravity-model bias does not leave the tool off target.
            constexpr double deadband_tolerance = 0.0005;
            double candidate_integral = integral_errors_[i];
            if (std::abs(pos_error) > deadband_tolerance) {
                candidate_integral = std::clamp(
                    candidate_integral + pos_error * dt, -i_clamp, i_clamp);
            }
            double raw_effort =
                (kp * pos_error) + (ki * candidate_integral) + (kd * vel_error) + ff_gravity;
            const bool saturated_high = raw_effort > limit && pos_error > 0.0;
            const bool saturated_low = raw_effort < -limit && pos_error < 0.0;
            if (!saturated_high && !saturated_low) {
                integral_errors_[i] = candidate_integral;
            } else {
                raw_effort =
                    (kp * pos_error) + (ki * integral_errors_[i]) + (kd * vel_error) + ff_gravity;
            }
            effort_msg.data[i] = std::clamp(raw_effort, -limit, limit);
        }

        if (trajectory_active_)
        {
            bool all_joints_settled = true;

            constexpr double settle_position_tol = 0.005;
            constexpr double settle_velocity_tol = 0.01;

            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                double pos_error =
                    std::abs(target_positions_[i] - current_positions_[i]);

                double vel =
                    std::abs(current_velocities_[i]);

                if (pos_error > settle_position_tol ||
                    vel > settle_velocity_tol)
                {
                    all_joints_settled = false;
                    break;
                }
            }

            if (all_joints_settled)
            {
                settled_counter_++;
            }
            else
            {
                settled_counter_ = 0;
            }

            if (settled_counter_ >= 250)
            {
                // Stop trajectory interpolation but keep the final reference active for position holding.
                trajectory_active_ = false;
                settled_counter_ = 0;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Trajectory completed. Arm settled and locked.");
            }
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
    std::vector<double> smoothed_vel_errors_;
    std::vector<bool> joint_state_seen_;
    trajectory_msgs::msg::JointTrajectory active_trajectory_;
    rclcpp::Time trajectory_start_time_;
    rclcpp::Time last_time_;
    bool trajectory_active_;
    bool target_initialized_;
    int settled_counter_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Add the class type inside the make_shared template angle brackets
    rclcpp::spin(std::make_shared<CompletePIDGravityController>());
    rclcpp::shutdown();
    return 0;
}
