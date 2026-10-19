#include <memory>
#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

constexpr int NUM_JOINTS = 6;
constexpr double PI = 3.14159265358979323846;

struct Input {
    double Vmax, Amax, Jmax, t;
    double XStart, XGoal, MS, GR;
};

struct Output {
    double S, X, V, A, J, DT;
};

struct Phases {
    double T1{}, T2{}, T3{}, T4{}, T5{}, T6{}, T7{};
    double Total{}, Vpeak{}, Apeak{};
    bool VmaxReached{}, AmaxReached{};
};

class TrajectoryPlannerNode : public rclcpp::Node {
public:
    TrajectoryPlannerNode() : Node("trajectory_planner_node") {
        raw_joint_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/ik_node/raw_joint_targets", 10, std::bind(&TrajectoryPlannerNode::ik_callback, this, std::placeholders::_1));

        // NEW: Subscribe to the joint states outputted by your broadcaster
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&TrajectoryPlannerNode::joint_state_callback, this, std::placeholders::_1));

        smooth_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);

        // Initialize state to 0 safely
        latest_joint_positions_.fill(0.0);

        RCLCPP_INFO(this->get_logger(), "7-Phase S-Curve Joint Trajectory Node is online.");
    }

private:
    double clamp(double x, double lo, double hi) { return std::max(lo, std::min(x, hi)); }

    Phases CalculatePhases(double S, double Vmax, double Amax, double Jmax) {
        Phases p{};
        if (S <= 0 || Vmax <= 0 || Amax <= 0 || Jmax <= 0) return p;
        double Va = Amax * Amax / Jmax;
        if (Vmax < Va) {
            p.AmaxReached = false; p.Apeak = std::sqrt(Vmax * Jmax);
            p.T1 = std::sqrt(Vmax / Jmax); p.T2 = 0; p.T3 = p.T1; p.Vpeak = Vmax;
        } else {
            p.AmaxReached = true; p.Apeak = Amax;
            p.T1 = Amax / Jmax; p.T2 = Vmax / Amax - p.T1; p.T3 = p.T1; p.Vpeak = Vmax;
        }
        double Sacc = 0.5 * p.Apeak * (p.T2 * p.T2 + 3 * p.T2 * p.T1 + 2 * p.T1 * p.T1);
        double SaccDec = 2 * Sacc;
        if (S >= SaccDec) {
            p.VmaxReached = true; p.T4 = (S - SaccDec) / p.Vpeak;
        } else {
            p.VmaxReached = false; p.T4 = 0;
            double TjMax = Amax / Jmax;
            double threshold = 2 * Jmax * TjMax * TjMax * TjMax;
            if (S < threshold) {
                double Tj = std::cbrt(S / (2 * Jmax));
                p.T1 = p.T3 = Tj; p.T2 = 0; p.Apeak = Jmax * Tj; p.Vpeak = Jmax * Tj * Tj; p.AmaxReached = false;
            } else {
                p.T1 = p.T3 = TjMax;
                p.T2 = (-3 * TjMax + std::sqrt(TjMax * TjMax + 4 * S / Amax)) / 2;
                p.T2 = std::max(0.0, p.T2); p.Apeak = Amax; p.Vpeak = Amax * (p.T1 + p.T2); p.AmaxReached = true;
            }
        }
        p.T5 = p.T1; p.T6 = p.T2; p.T7 = p.T3;
        p.Total = p.T1 + p.T2 + p.T3 + p.T4 + p.T5 + p.T6 + p.T7;
        return p;
    }

    Output Evaluate(const Input& in, const Phases& p) {
        Output o{};
        double S = std::abs(in.XGoal - in.XStart);
        double dir = (in.XGoal >= in.XStart) ? 1.0 : -1.0;
        double t = clamp(in.t, 0, p.Total);
        double j = in.Jmax;
        double T1 = p.T1; double T2 = p.T2; double T3 = p.T3; double T4 = p.T4;
        double T5 = p.T5; double T6 = p.T6; double T7 = p.T7;
        double A = p.Apeak; double V = p.Vpeak;
        double s = 0, v = 0, a = 0, jerk = 0;

        if (t <= T1) { jerk = j; a = j * t; v = 0.5 * j * t * t; s = j * t * t * t / 6; }
        else if (t <= T1 + T2) { double q = t - T1; jerk = 0; a = A; double v1 = 0.5 * j * T1 * T1; double s1 = j * T1 * T1 * T1 / 6; v = v1 + A * q; s = s1 + v1 * q + 0.5 * A * q * q; }
        else if (t <= T1 + T2 + T3) { double q = t - T1 - T2; double v1 = 0.5 * j * T1 * T1; double v2 = v1 + A * T2; double s1 = j * T1 * T1 * T1 / 6; double s2 = s1 + v1 * T2 + 0.5 * A * T2 * T2; jerk = -j; a = A - j * q; v = v2 + A * q - 0.5 * j * q * q; s = s2 + v2 * q + 0.5 * A * q * q - j * q * q * q / 6; }
        else if (t <= T1 + T2 + T3 + T4) { double q = t - T1 - T2 - T3; jerk = 0; a = 0; v = V; double Sacc = 0.5 * A * (T2 * T2 + 3 * T2 * T1 + 2 * T1 * T1); s = Sacc + V * q; }
        else if (t <= T1 + T2 + T3 + T4 + T5) { double q = t - T1 - T2 - T3 - T4; jerk = -j; a = -j * q; v = V - 0.5 * j * q * q; double Sacc = 0.5 * A * (T2 * T2 + 3 * T2 * T1 + 2 * T1 * T1); s = Sacc + V * T4 + V * q - j * q * q * q / 6; }
        else if (t <= T1 + T2 + T3 + T4 + T5 + T6) { double q = t - T1 - T2 - T3 - T4 - T5; jerk = 0; a = -A; double v5 = V - 0.5 * j * T5 * T5; double Sacc = 0.5 * A * (T2 * T2 + 3 * T2 * T1 + 2 * T1 * T1); double s5 = Sacc + V * T4 + V * T5 - j * T5 * T5 * T5 / 6; v = v5 - A * q; s = s5 + v5 * q - 0.5 * A * q * q; }
        else { double q = t - T1 - T2 - T3 - T4 - T5 - T6; jerk = j; a = -A + j * q; double Sacc = 0.5 * A * (T2 * T2 + 3 * T2 * T1 + 2 * T1 * T1); double Sconst = V * T4; double v5 = V - 0.5 * j * T5 * T5; double S5 = V * T5 - j * T5 * T5 * T5 / 6; double S6 = v5 * T6 - 0.5 * A * T6 * T6; double v7 = A * T7 - 0.5 * j * T7 * T7; double start = Sacc + Sconst + S5 + S6; v = v7 - A * q + 0.5 * j * q * q; s = start + v7 * q - 0.5 * A * q * q + j * q * q * q / 6; }

        if (in.t >= p.Total) { s = S; v = a = jerk = 0; }
        o.S = dir * s; o.X = in.XStart + o.S; o.V = dir * v; o.A = dir * a; o.J = dir * jerk;
        return o;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::vector<std::string> target_order = {
            "rail_joint", "base_joint", "shoulder_joint", 
            "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"
        };

        // Match joint names safely to maintain axis ordering
        for (size_t i = 0; i < target_order.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), target_order[i]);
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                if (index < msg->position.size()) {
                    latest_joint_positions_[i] = msg->position[index];
                }
            }
        }
    }

    void ik_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr raw_msg) {
        if (raw_msg->points.empty()) return;
        if (raw_msg->points[0].positions.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Incoming IK message has fewer than 6 joints. Processing fallback.");
        }

        auto smooth_msg = trajectory_msgs::msg::JointTrajectory();
        smooth_msg.header.stamp = this->get_clock()->now();
        smooth_msg.joint_names = {
            "rail_joint", "base_joint", "shoulder_joint", 
            "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"
        };

        std::array<double, NUM_JOINTS> Vmax = {0.3, 1.0, 1.0, 1.0, 1.5, 1.5};
        std::array<double, NUM_JOINTS> Amax = {0.5, 2.0, 2.0, 2.0, 3.0, 3.0};
        std::array<double, NUM_JOINTS> Jmax = {1.0, 5.0, 5.0, 5.0, 8.0, 8.0};

        // replace hardcoded zeros with real positions
        std::array<double, NUM_JOINTS> current_positions = latest_joint_positions_;
        std::array<double, NUM_JOINTS> goal_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Securely copy the incoming positions from raw_msg
        for (size_t i = 0; i < NUM_JOINTS; ++i) {
            if (i < raw_msg->points[0].positions.size()) {
                goal_positions[i] = raw_msg->points[0].positions[i];
            }
        }

        // Calculate S-curve total phase durations
        double max_total_time = 0.0;
        std::array<Phases, NUM_JOINTS> profiles;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            double distance = std::abs(goal_positions[i] - current_positions[i]);
            profiles[i] = CalculatePhases(distance, Vmax[i], Amax[i], Jmax[i]);
            if (profiles[i].Total > max_total_time) {
                max_total_time = profiles[i].Total;
            }
        }

        // CRITICAL ADAPTATION FOR GAZEBO: Add an explicit t=0 entry point to lock the trajectory start state
        auto start_point = trajectory_msgs::msg::JointTrajectoryPoint();
        start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
        start_point.positions = {current_positions[0], current_positions[1], current_positions[2], current_positions[3], current_positions[4], current_positions[5]};
        start_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        start_point.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        smooth_msg.points.push_back(start_point);

        // Sample the synchronous pathway at 100Hz frequency intervals (dt = 0.01 seconds)
        double dt = 0.01;
        for (double t = dt; t <= max_total_time; t += dt) {
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            point.time_from_start = rclcpp::Duration::from_seconds(t);

            for (int i = 0; i < NUM_JOINTS; ++i) {
                Input in{Vmax[i], Amax[i], Jmax[i], t, current_positions[i], goal_positions[i], 16.0, 10.0};
                Output state = Evaluate(in, profiles[i]);
                point.positions.push_back(state.X);
                point.velocities.push_back(state.V);
                point.accelerations.push_back(state.A);
            }
            smooth_msg.points.push_back(point);
        }

        if (!smooth_msg.points.empty()) {
            auto & final_point = smooth_msg.points.back();
            std::fill(final_point.velocities.begin(), final_point.velocities.end(), 0.0);
            std::fill(final_point.accelerations.begin(), final_point.accelerations.end(), 0.0);
        }
        
        smooth_trajectory_pub_->publish(smooth_msg);
        RCLCPP_INFO(this->get_logger(), "Synchronized smooth S-Curve trajectory successfully broadcasted over %.2f seconds.", max_total_time);
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr raw_joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr smooth_trajectory_pub_;
    std::array<double, NUM_JOINTS> latest_joint_positions_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
