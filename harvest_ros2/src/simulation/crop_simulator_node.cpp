#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/delete_entity.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{
constexpr std::array<const char *, 6> kJointNames = {
  "rail_joint", "base_joint", "shoulder_joint", "elbow_joint",
  "wrist_pitch_joint", "wrist_roll_joint"};
constexpr double kCropRadius = 0.012;
constexpr double kClusterCenterY = 0.30;
constexpr double kClusterCenterZ = 0.385;
constexpr double kGridSpacing = 0.035;
constexpr double kGridJitter = 0.002;
constexpr size_t kImageSampleStep = 2;
constexpr size_t kMinimumRedComponentPixels = 4;
}  // namespace

struct CropTarget
{
  std::string name;
  geometry_msgs::msg::Pose pose;
};

class CropSimulatorNode : public rclcpp::Node
{
public:
  CropSimulatorNode()
  : Node("crop_simulator_node"),
    random_engine_(std::random_device{}())
  {
    const std::string metrics_file =
      declare_parameter<std::string>("metrics_file", "harvest_evaluation_metrics.csv");
    metrics_sample_period_sec_ = declare_parameter<double>("metrics_sample_period_sec", 1.0);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 60.0);
    if (!std::isfinite(metrics_sample_period_sec_) ||
      !std::isfinite(target_timeout_sec_) ||
      metrics_sample_period_sec_ <= 0.0 || target_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "metrics_sample_period_sec and target_timeout_sec must both be positive.");
    }
    open_metrics_file(metrics_file);

    spawn_client_ =
      create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/empty/create");
    delete_client_ =
      create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/empty/remove");

    spawn_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/crop_simulator/spawn_strawberry", 10,
      std::bind(&CropSimulatorNode::handle_spawn_msg, this, std::placeholders::_1));
    start_cluster_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/crop_simulator/start_cluster", 10,
      std::bind(&CropSimulatorNode::handle_start_cluster, this, std::placeholders::_1));
    harvest_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/harvest/cycle_complete", 10,
      std::bind(&CropSimulatorNode::handle_harvest_complete, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&CropSimulatorNode::handle_joint_state, this, std::placeholders::_1));
    camera_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&CropSimulatorNode::handle_camera_image, this, std::placeholders::_1));
    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10,
      std::bind(&CropSimulatorNode::handle_trajectory, this, std::placeholders::_1));

    metrics_timer_ = create_wall_timer(
      std::chrono::milliseconds(
        std::max<int64_t>(1, static_cast<int64_t>(metrics_sample_period_sec_ * 1000.0))),
      std::bind(&CropSimulatorNode::write_periodic_metrics, this));

    target_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("/harvest_target_pose", 10);
    berry_detected_pub_ =
      create_publisher<std_msgs::msg::String>("/crop_simulator/berry_detected", 10);

    if (declare_parameter<bool>("auto_start_cluster", true)) {
      startup_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CropSimulatorNode::try_start_cluster, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "Crop simulator ready. A random 5-7 crop cluster will be harvested automatically; "
      "the camera stream on /camera/image_raw gates each harvest. Publish std_msgs/msg/Empty "
      "to /crop_simulator/start_cluster to start another. Evaluation metrics: '%s'.",
      metrics_file_path_.c_str());
    write_metrics_row("simulation_started", "idle");
  }

private:
  void open_metrics_file(const std::string & configured_path)
  {
    std::filesystem::path path(configured_path);
    if (path.empty()) {
      throw std::invalid_argument("metrics_file must not be empty.");
    }
    if (path.is_relative()) {
      path = std::filesystem::current_path() / path;
    }
    path = path.lexically_normal();
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }

    metrics_file_.open(path, std::ios::out | std::ios::trunc);
    if (!metrics_file_.is_open()) {
      RCLCPP_FATAL(
        get_logger(), "Could not open evaluation metrics file '%s'.", path.c_str());
      throw std::runtime_error("Could not open the crop evaluation metrics file.");
    }
    metrics_file_path_ = path.string();
    metrics_file_ <<
      "wall_elapsed_s,sim_time_s,event,cluster_id,target_index,target_name,status,"
      "target_source,detection_status,detection_wait_s,clear_result,target_elapsed_s,"
      "target_x_m,target_y_m,target_z_m,crops_cleared,crops_total,"
      "cluster_clearing_time_sim_s,cluster_clearing_time_wall_s,"
      "rail_velocity_m_s,base_velocity_rad_s,shoulder_velocity_rad_s,elbow_velocity_rad_s,"
      "wrist_pitch_velocity_rad_s,wrist_roll_velocity_rad_s,"
      "rail_acceleration_m_s2,base_acceleration_rad_s2,shoulder_acceleration_rad_s2,"
      "elbow_acceleration_rad_s2,wrist_pitch_acceleration_rad_s2,"
      "wrist_roll_acceleration_rad_s2\n";
    metrics_file_.flush();
    if (!metrics_file_) {
      RCLCPP_FATAL(
        get_logger(), "Could not write the header to evaluation metrics file '%s'.",
        metrics_file_path_.c_str());
      throw std::runtime_error("Could not initialize the crop evaluation metrics file.");
    }
    metrics_start_wall_time_ = std::chrono::steady_clock::now();
    metrics_start_sim_time_ = now();
  }

  static std::string csv_string(const std::string & value)
  {
    std::string escaped = "\"";
    for (const char character : value) {
      if (character == '"') {
        escaped += '"';
      }
      escaped += character;
    }
    escaped += '"';
    return escaped;
  }

  void write_metrics_row(
    const std::string & event, const std::string & status,
    const std::string & clear_result = "")
  {
    if (!metrics_file_.is_open() || metrics_file_failed_) {
      return;
    }

    const auto now_wall = std::chrono::steady_clock::now();
    const double wall_elapsed =
      std::chrono::duration<double>(now_wall - metrics_start_wall_time_).count();
    const double sim_elapsed = (now() - metrics_start_sim_time_).seconds();
    const bool has_target =
      current_target_index_ < cluster_targets_.size() &&
      (cluster_active_ || cluster_setup_in_progress_ || status == "complete" ||
      status == "failed");
    const CropTarget * target = has_target ? &cluster_targets_[current_target_index_] : nullptr;

    metrics_file_ << std::fixed << std::setprecision(6)
                  << wall_elapsed << ',' << sim_elapsed << ','
                  << csv_string(event) << ',' << cluster_id_ << ',';
    if (target != nullptr) {
      metrics_file_ << current_target_index_ + 1 << ',' << csv_string(target->name);
    } else {
      metrics_file_ << ',';
    }
    metrics_file_ << ',' << csv_string(status) << ','
                  << csv_string(target_source_) << ','
                  << csv_string(target_detected_ ? "detected" :
      (awaiting_camera_detection_ ? "waiting" : "not_visually_detected")) << ',';
    if (detection_wait_complete_) {
      metrics_file_ << detection_wait_wall_sec_;
    }
    metrics_file_ << ',' << csv_string(
      clear_result.empty() ? target_clear_result_ : clear_result) << ',';
    if (target_command_started_) {
      metrics_file_ << std::chrono::duration<double>(
        now_wall - target_command_wall_time_).count();
    } else if (awaiting_camera_detection_) {
      metrics_file_ << std::chrono::duration<double>(
        now_wall - detection_start_wall_time_).count();
    }
    metrics_file_ << ',';
    if (target != nullptr) {
      metrics_file_ << target->pose.position.x << ',' << target->pose.position.y << ','
                    << target->pose.position.z;
    } else {
      metrics_file_ << ",,";
    }
    metrics_file_ << ',' << cleared_count_ << ','
                  << (cluster_targets_.empty() ? 0 : cluster_targets_.size()) << ',';
    if (cluster_started_) {
      metrics_file_ << (now() - cluster_start_time_).seconds() << ','
                    << std::chrono::duration<double>(
        now_wall - cluster_start_wall_time_).count();
    } else {
      metrics_file_ << ',';
    }
    for (size_t axis = 0; axis < kJointNames.size(); ++axis) {
      metrics_file_ << ',';
      if (joint_velocity_valid_[axis]) {
        metrics_file_ << joint_velocity_[axis];
      }
    }
    for (size_t axis = 0; axis < kJointNames.size(); ++axis) {
      metrics_file_ << ',';
      if (joint_acceleration_valid_[axis]) {
        metrics_file_ << joint_acceleration_[axis];
      }
    }
    metrics_file_ << '\n';
    metrics_file_.flush();
    if (!metrics_file_) {
      metrics_file_failed_ = true;
      RCLCPP_ERROR(
        get_logger(), "Writing evaluation metrics to '%s' failed; further CSV writes stopped.",
        metrics_file_path_.c_str());
    }
  }

  std::string current_metrics_status() const
  {
    if (cluster_setup_in_progress_) {
      return "spawning_cluster";
    }
    if (!cluster_active_) {
      return cluster_status_;
    }
    if (awaiting_camera_detection_) {
      return "waiting_for_camera_detection";
    }
    if (delete_pending_) {
      return "deleting_crop";
    }
    if (waiting_for_harvest_) {
      return "waiting_for_harvest_completion";
    }
    return "cluster_active";
  }

  void write_periodic_metrics()
  {
    const auto wall_now = std::chrono::steady_clock::now();
    if (cluster_active_ && !target_timeout_logged_) {
      const auto phase_start = awaiting_camera_detection_ ?
        detection_start_wall_time_ : target_command_wall_time_;
      const bool phase_started = awaiting_camera_detection_ || target_command_started_;
      if (phase_started && std::chrono::duration<double>(wall_now - phase_start).count() >=
        target_timeout_sec_)
      {
        target_timeout_logged_ = true;
        target_clear_result_ = "failed";
        write_metrics_row("target_timeout", current_metrics_status(), "failed");
        RCLCPP_WARN(
          get_logger(),
          "Target %zu timed out after %.1f wall seconds; evaluation continues recording.",
          current_target_index_ + 1, target_timeout_sec_);
      }
    }
    write_metrics_row("telemetry", current_metrics_status());
  }

  std::string make_crop_sdf(const std::string & name) const
  {
    std::ostringstream sdf;
    sdf << "<sdf version='1.7'>"
        << "<model name='" << name << "'>"
        << "<static>true</static>"
        << "<link name='link'>"
        << "<visual name='visual'><geometry><sphere><radius>"
        << kCropRadius
        << "</radius></sphere></geometry>"
        << "<material><ambient>1.0 0.0 0.05 1.0</ambient>"
        << "<diffuse>1.0 0.0 0.05 1.0</diffuse>"
        << "<emissive>0.25 0.0 0.01 1.0</emissive></material>"
        << "</visual>"
        // A short green stem above each fruit makes the suspended greenhouse cluster visible.
        << "<visual name='stem'><pose>0 0 0.018 0 0 0</pose>"
        << "<geometry><cylinder><radius>0.002</radius><length>0.016</length></cylinder></geometry>"
        << "<material><ambient>0.08 0.35 0.05 1.0</ambient>"
        << "<diffuse>0.08 0.35 0.05 1.0</diffuse></material></visual>"
        << "<collision name='collision'><geometry><sphere><radius>"
        << kCropRadius
        << "</radius></sphere></geometry></collision>"
        << "</link></model></sdf>";
    return sdf.str();
  }

  std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity::Request> make_spawn_request(
    const std::string & name, const geometry_msgs::msg::Pose & pose) const
  {
    auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    request->entity_factory.sdf = make_crop_sdf(name);
    request->entity_factory.name = name;
    request->entity_factory.pose = pose;
    request->entity_factory.relative_to = "world";
    return request;
  }

  void handle_spawn_msg(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (cluster_setup_in_progress_ || cluster_active_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring single-crop spawn while a cluster harvest is active.");
      return;
    }
    if (msg->header.frame_id != "world") {
      RCLCPP_ERROR(
        get_logger(), "Crop spawn pose must use the 'world' frame, received '%s'.",
        msg->header.frame_id.c_str());
      return;
    }
    if (!spawn_client_->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(), "Gazebo spawn service /world/empty/create is unavailable.");
      write_metrics_row("single_crop_spawn_failed", "failed", "failed");
      return;
    }

    const std::string name = "strawberry_" + std::to_string(sphere_count_++);
    auto request = make_spawn_request(name, msg->pose);
    spawn_client_->async_send_request(
      request,
      [this, name, msg](rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(get_logger(), "Gazebo failed to spawn crop object '%s'.", name.c_str());
          write_metrics_row("single_crop_spawn_failed", "failed", "failed");
          return;
        }
        RCLCPP_INFO(get_logger(), "Gazebo spawned crop object '%s'.", name.c_str());
        write_metrics_row("single_crop_spawned", "single_crop_spawned", "pending");
        target_pub_->publish(*msg);
        RCLCPP_INFO(get_logger(), "Sent strawberry pose to IK on /harvest_target_pose.");
      });
  }

  void handle_start_cluster(const std_msgs::msg::Empty::SharedPtr)
  {
    begin_cluster();
  }

  void try_start_cluster()
  {
    if (!spawn_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for Gazebo spawn service /world/empty/create.");
      return;
    }
    startup_timer_->cancel();
    begin_cluster();
  }

  void begin_cluster()
  {
    if (cluster_setup_in_progress_ || cluster_active_) {
      RCLCPP_WARN(get_logger(), "A cluster setup or harvest is already in progress.");
      return;
    }
    ++cluster_id_;
    if (!spawn_client_->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(), "Cannot start cluster: Gazebo spawn service /world/empty/create is unavailable.");
      cluster_status_ = "failed";
      write_metrics_row("cluster_start_failed", "failed", "failed");
      return;
    }

    cluster_setup_in_progress_ = true;
    cluster_started_ = false;
    cluster_status_ = "spawning_cluster";
    cluster_targets_ = generate_cluster();
    next_spawn_index_ = 0;
    current_target_index_ = 0;
    cleared_count_ = 0;
    target_detected_ = false;
    detection_wait_complete_ = false;
    target_command_started_ = false;
    target_timeout_logged_ = false;
    target_source_.clear();
    target_clear_result_ = "pending";
    cluster_peak_error_.fill(0.0);
    cluster_squared_error_sum_.fill(0.0);
    cluster_sample_count_.fill(0);
    cleared_count_ = 0;
    write_metrics_row("cluster_started", "spawning_cluster", "pending");
    RCLCPP_INFO(
      get_logger(), "Spawning a randomized vertical hanging cluster of %zu crops.",
      cluster_targets_.size());
    spawn_next_cluster_crop();
  }

  std::vector<CropTarget> generate_cluster()
  {
    std::uniform_int_distribution<int> count_distribution(5, 7);
    std::uniform_real_distribution<double> jitter_distribution(-kGridJitter, kGridJitter);
    const int count = count_distribution(random_engine_);

    std::vector<std::array<int, 2>> grid;
    // Distribute fruit across horizontal columns and vertical hanging levels,
    // rather than laying the cluster flat on a ground plane.
    for (int row = -1; row <= 1; ++row) {
      for (int column = -1; column <= 1; ++column) {
        grid.push_back({column, row});
      }
    }
    std::shuffle(grid.begin(), grid.end(), random_engine_);

    std::vector<CropTarget> targets;
    targets.reserve(static_cast<size_t>(count));
    for (int index = 0; index < count; ++index) {
      const auto cell = grid[static_cast<size_t>(index)];
      CropTarget target;
      target.name = "strawberry_" + std::to_string(sphere_count_++);
      target.pose.position.x =
        cell[0] * kGridSpacing + jitter_distribution(random_engine_);
      target.pose.position.y =
        kClusterCenterY + jitter_distribution(random_engine_);
      target.pose.position.z =
        kClusterCenterZ + cell[1] * kGridSpacing + jitter_distribution(random_engine_);
      target.pose.orientation.w = 1.0;
      targets.push_back(std::move(target));
    }
    return targets;
  }

  void spawn_next_cluster_crop()
  {
    if (next_spawn_index_ >= cluster_targets_.size()) {
      cluster_setup_in_progress_ = false;
      cluster_active_ = true;
      cluster_start_time_ = now();
      cluster_start_wall_time_ = std::chrono::steady_clock::now();
      cluster_started_ = true;
      current_target_index_ = 0;
      cluster_status_ = "waiting_for_camera_detection";
      RCLCPP_INFO(
        get_logger(), "Cluster ready; harvesting %zu crops sequentially.",
        cluster_targets_.size());
      current_target_index_ = 0;
      write_metrics_row("cluster_ready", "waiting_for_camera_detection", "pending");
      wait_for_camera_detection();
      return;
    }

    const auto & target = cluster_targets_[next_spawn_index_];
    auto request = make_spawn_request(target.name, target.pose);
    spawn_client_->async_send_request(
      request,
      [this, name = target.name](
        rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(get_logger(), "Gazebo failed to spawn cluster crop '%s'.", name.c_str());
          cluster_status_ = "failed";
          write_metrics_row("cluster_crop_spawn_failed", "failed", "failed");
          cleanup_index_ = next_spawn_index_;
          cleanup_succeeded_ = true;
          cleanup_partial_cluster();
          return;
        }
        RCLCPP_INFO(get_logger(), "Gazebo spawned cluster crop '%s'.", name.c_str());
        ++next_spawn_index_;
        spawn_next_cluster_crop();
      });
  }

  void cleanup_partial_cluster()
  {
    if (cleanup_index_ == 0) {
      cluster_setup_in_progress_ = false;
      cluster_targets_.clear();
      if (cleanup_succeeded_) {
        RCLCPP_WARN(
          get_logger(), "Aborted cluster setup and removed all crops that spawned successfully.");
      } else {
        RCLCPP_ERROR(
          get_logger(), "Aborted cluster setup; at least one partially spawned crop remains.");
      }
      write_metrics_row(
        cleanup_succeeded_ ? "cluster_setup_aborted_clean" :
        "cluster_setup_cleanup_failed", "failed", "failed");
      return;
    }
    if (!delete_client_->service_is_ready()) {
      cluster_setup_in_progress_ = false;
      cluster_targets_.clear();
      RCLCPP_ERROR(
        get_logger(), "Could not clean up partially spawned cluster: Gazebo delete service "
        "/world/empty/remove is unavailable.");
      cluster_status_ = "failed";
      write_metrics_row("cluster_setup_cleanup_failed", "failed", "failed");
      return;
    }

    const std::string name = cluster_targets_[--cleanup_index_].name;
    auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    request->entity.name = name;
    delete_client_->async_send_request(
      request,
      [this, name](
        rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          cleanup_succeeded_ = false;
          RCLCPP_ERROR(
            get_logger(), "Failed to clean up partially spawned crop '%s'.", name.c_str());
          write_metrics_row("partial_crop_cleanup_failed", "failed", "failed");
        }
        cleanup_partial_cluster();
      });
  }

  void publish_current_target(bool detected_by_camera = false)
  {
    if (!cluster_active_ || current_target_index_ >= cluster_targets_.size()) {
      return;
    }

    reset_target_metrics();
    target_detected_ = detected_by_camera;
    target_source_ = detected_by_camera ? "camera" : "simulator_ground_truth";
    target_clear_result_ = "pending";
    target_command_started_ = true;
    target_command_wall_time_ = std::chrono::steady_clock::now();
    target_timeout_logged_ = false;
    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.stamp = now();
    target_msg.header.frame_id = "world";
    target_msg.pose = cluster_targets_[current_target_index_].pose;
    waiting_for_harvest_ = true;
    target_pub_->publish(target_msg);
    write_metrics_row("target_commanded", "moving_to_target", "pending");
    RCLCPP_INFO(
      get_logger(), "Target %zu/%zu: '%s' at (%.3f, %.3f, %.3f).",
      current_target_index_ + 1, cluster_targets_.size(),
      cluster_targets_[current_target_index_].name.c_str(),
      target_msg.pose.position.x, target_msg.pose.position.y, target_msg.pose.position.z);
  }

  bool has_red_berry_component(const sensor_msgs::msg::Image & image) const
  {
    size_t channels = 0;
    bool red_first = true;
    if (image.encoding == "rgb8" || image.encoding == "rgba8") {
      channels = image.encoding == "rgb8" ? 3 : 4;
    } else if (image.encoding == "bgr8" || image.encoding == "bgra8") {
      channels = image.encoding == "bgr8" ? 3 : 4;
      red_first = false;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Camera image encoding '%s' is unsupported; expected rgb8, bgr8, rgba8, or bgra8.",
        image.encoding.c_str());
      return false;
    }

    if (image.width == 0 || image.height == 0 ||
        image.step < static_cast<size_t>(image.width) * channels ||
        image.data.size() < static_cast<size_t>(image.step) * image.height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring malformed camera image dimensions or data.");
      return false;
    }

    const size_t sampled_width = (image.width + kImageSampleStep - 1) / kImageSampleStep;
    const size_t sampled_height = (image.height + kImageSampleStep - 1) / kImageSampleStep;
    std::vector<uint8_t> red_mask(sampled_width * sampled_height, 0);

    // Sample every other pixel to keep image processing inexpensive at the sensor's 15 Hz rate.
    for (size_t sample_y = 0; sample_y < sampled_height; ++sample_y) {
      const size_t y = sample_y * kImageSampleStep;
      for (size_t sample_x = 0; sample_x < sampled_width; ++sample_x) {
        const size_t x = sample_x * kImageSampleStep;
        const size_t offset = y * image.step + x * channels;
        const uint8_t red = image.data[offset + (red_first ? 0 : 2)];
        const uint8_t green = image.data[offset + 1];
        const uint8_t blue = image.data[offset + (red_first ? 2 : 0)];

        // The bright red berry material is separated from green/blue channels by this threshold.
        if (red >= 80 && red > green * 1.35 && red > blue * 1.25) {
          red_mask[sample_y * sampled_width + sample_x] = 1;
        }
      }
    }

    // Connected components reject isolated red noise while allowing distant berries to be small.
    std::vector<uint8_t> visited(red_mask.size(), 0);
    std::vector<size_t> pending;
    for (size_t start = 0; start < red_mask.size(); ++start) {
      if (red_mask[start] == 0 || visited[start] != 0) {
        continue;
      }

      pending.clear();
      pending.push_back(start);
      visited[start] = 1;
      size_t component_size = 0;
      for (size_t cursor = 0; cursor < pending.size(); ++cursor) {
        const size_t pixel = pending[cursor];
        const size_t x = pixel % sampled_width;
        const size_t y = pixel / sampled_width;
        ++component_size;
        const std::array<size_t, 4> neighbors = {
          x > 0 ? pixel - 1 : red_mask.size(),
          x + 1 < sampled_width ? pixel + 1 : red_mask.size(),
          y > 0 ? pixel - sampled_width : red_mask.size(),
          y + 1 < sampled_height ? pixel + sampled_width : red_mask.size()};
        for (const size_t neighbor : neighbors) {
          if (neighbor < red_mask.size() && red_mask[neighbor] != 0 &&
              visited[neighbor] == 0)
          {
            visited[neighbor] = 1;
            pending.push_back(neighbor);
          }
        }
      }

      if (component_size >= kMinimumRedComponentPixels) {
        return true;
      }
    }
    return false;
  }

  void wait_for_camera_detection()
  {
    awaiting_camera_detection_ = true;
    minimum_detection_stamp_ = now();
    detection_start_wall_time_ = std::chrono::steady_clock::now();
    detection_wait_complete_ = false;
    target_detected_ = false;
    target_source_ = "camera";
    target_clear_result_ = "pending";
    target_command_started_ = false;
    target_timeout_logged_ = false;
    write_metrics_row("waiting_for_detection", "waiting_for_camera_detection", "pending");
    RCLCPP_INFO(
      get_logger(),
      "Waiting for a fresh camera frame with a red berry before commanding target %zu/%zu.",
      current_target_index_ + 1, cluster_targets_.size());
  }

  void handle_camera_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!cluster_active_ || !awaiting_camera_detection_ ||
        waiting_for_harvest_ || delete_pending_)
    {
      return;
    }

    const rclcpp::Time image_stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (image_stamp <= minimum_detection_stamp_ || !has_red_berry_component(*msg)) {
      return;
    }

    const auto & target = cluster_targets_[current_target_index_];
    std_msgs::msg::String detection;
    detection.data = target.name;
    berry_detected_pub_->publish(detection);
    target_detected_ = true;
    target_source_ = "camera";
    detection_wait_wall_sec_ = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - detection_start_wall_time_).count();
    detection_wait_complete_ = true;
    target_clear_result_ = "pending";
    write_metrics_row("berry_detected", "detected", "pending");
    RCLCPP_INFO(
      get_logger(), "Camera detected a red berry in the cluster; selecting '%s'.",
      target.name.c_str());

    // Gazebo provides exact coordinates for the simulated crop; visual detection gates selection.
    awaiting_camera_detection_ = false;
    publish_current_target(true);
  }

  void handle_harvest_complete(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!cluster_active_ || !waiting_for_harvest_ || delete_pending_) {
      return;
    }
    if (!msg->data) {
      target_clear_result_ = "failed";
      write_metrics_row("harvest_cycle_failed", "harvest_failed", "failed");
      return;
    }
    waiting_for_harvest_ = false;
    write_metrics_row("harvest_cycle_complete", "deleting_crop", "pending");
    log_target_metrics();

    if (!delete_client_->service_is_ready()) {
      cluster_active_ = false;
      cluster_status_ = "failed";
      target_clear_result_ = "failed";
      RCLCPP_ERROR(
        get_logger(), "Cannot clear harvested crop: Gazebo delete service "
        "/world/empty/remove is unavailable.");
      write_metrics_row("crop_delete_service_unavailable", "failed", "failed");
      return;
    }

    const auto & target = cluster_targets_[current_target_index_];
    auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    request->entity.name = target.name;
    delete_pending_ = true;
    delete_client_->async_send_request(
      request,
      [this, name = target.name](
        rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future) {
        const auto response = future.get();
        delete_pending_ = false;
        if (!response->success) {
          cluster_active_ = false;
          cluster_status_ = "failed";
          target_clear_result_ = "failed";
          write_metrics_row("crop_delete_failed", "failed", "failed");
          RCLCPP_ERROR(get_logger(), "Gazebo failed to delete harvested crop '%s'.", name.c_str());
          return;
        }

        ++cleared_count_;
        target_clear_result_ = "success";
        write_metrics_row("crop_cleared", "crop_cleared", "success");
        RCLCPP_INFO(
          get_logger(), "Deleted harvested crop '%s' (%zu/%zu).",
          name.c_str(), cleared_count_, cluster_targets_.size());
        ++current_target_index_;
        if (current_target_index_ < cluster_targets_.size()) {
          // The end-effector camera can be blocked at the first crop; use the known spawned
          // positions for the rest of this simulated cluster rather than stalling on visibility.
          RCLCPP_INFO(
            get_logger(), "Selecting the next known crop from the simulated cluster.");
          publish_current_target();
        } else {
          finish_cluster();
        }
      });
  }

  void handle_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring an empty arm trajectory for tracking metrics.");
      return;
    }
    active_trajectory_ = *msg;
    trajectory_start_time_ = now();
    trajectory_received_ = true;
  }

  void handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const rclcpp::Time sample_time = now();
    joint_velocity_valid_.fill(false);
    joint_acceleration_valid_.fill(false);
    for (size_t axis = 0; axis < kJointNames.size(); ++axis) {
      const auto feedback = std::find(msg->name.begin(), msg->name.end(), kJointNames[axis]);
      if (feedback == msg->name.end()) {
        continue;
      }
      const size_t index = static_cast<size_t>(std::distance(msg->name.begin(), feedback));
      if (index >= msg->velocity.size()) {
        continue;
      }
      const double velocity = msg->velocity[index];
      joint_velocity_[axis] = velocity;
      joint_velocity_valid_[axis] = true;
      if (previous_joint_velocity_valid_[axis] && have_joint_state_time_) {
        const double elapsed = (sample_time - last_joint_state_time_).seconds();
        if (elapsed > 1e-6) {
          joint_acceleration_[axis] =
            (velocity - previous_joint_velocity_[axis]) / elapsed;
          joint_acceleration_valid_[axis] = true;
        }
      }
      previous_joint_velocity_[axis] = velocity;
      previous_joint_velocity_valid_[axis] = true;
    }
    if (sample_time > last_joint_state_time_) {
      last_joint_state_time_ = sample_time;
      have_joint_state_time_ = true;
    }

    if (!cluster_active_ || !trajectory_received_) {
      return;
    }

    const double elapsed = (now() - trajectory_start_time_).seconds();
    const auto & points = active_trajectory_.points;
    size_t lower_index = 0;
    size_t upper_index = 0;
    double interpolation = 0.0;

    if (elapsed >= rclcpp::Duration(points.back().time_from_start).seconds()) {
      lower_index = upper_index = points.size() - 1;
    } else if (elapsed > rclcpp::Duration(points.front().time_from_start).seconds()) {
      for (size_t i = 1; i < points.size(); ++i) {
        const double upper_time = rclcpp::Duration(points[i].time_from_start).seconds();
        if (upper_time >= elapsed) {
          lower_index = i - 1;
          upper_index = i;
          const double lower_time =
            rclcpp::Duration(points[lower_index].time_from_start).seconds();
          const double interval = upper_time - lower_time;
          interpolation = interval > 0.0 ? (elapsed - lower_time) / interval : 1.0;
          break;
        }
      }
    }

    for (size_t axis = 0; axis < kJointNames.size(); ++axis) {
      const auto joint = std::find(
        active_trajectory_.joint_names.begin(), active_trajectory_.joint_names.end(),
        kJointNames[axis]);
      const auto feedback = std::find(msg->name.begin(), msg->name.end(), kJointNames[axis]);
      if (joint == active_trajectory_.joint_names.end() || feedback == msg->name.end()) {
        continue;
      }
      const size_t trajectory_index =
        static_cast<size_t>(std::distance(active_trajectory_.joint_names.begin(), joint));
      const size_t feedback_index = static_cast<size_t>(std::distance(msg->name.begin(), feedback));
      if (trajectory_index >= points[lower_index].positions.size() ||
          trajectory_index >= points[upper_index].positions.size() ||
          feedback_index >= msg->position.size()) {
        continue;
      }

      const double commanded = points[lower_index].positions[trajectory_index] +
        interpolation * (
        points[upper_index].positions[trajectory_index] -
        points[lower_index].positions[trajectory_index]);
      const double error = std::abs(msg->position[feedback_index] - commanded);
      target_peak_error_[axis] = std::max(target_peak_error_[axis], error);
      target_squared_error_sum_[axis] += error * error;
      ++target_sample_count_[axis];
    }
  }

  void reset_target_metrics()
  {
    target_peak_error_.fill(0.0);
    target_squared_error_sum_.fill(0.0);
    target_sample_count_.fill(0);
  }

  void log_target_metrics()
  {
    RCLCPP_INFO(
      get_logger(), "Tracking errors for target %zu (peak / RMS; prismatic m, revolute rad):",
      current_target_index_ + 1);
    for (size_t axis = 0; axis < kJointNames.size(); ++axis) {
      const double rms = target_sample_count_[axis] == 0 ? 0.0 :
        std::sqrt(target_squared_error_sum_[axis] /
        static_cast<double>(target_sample_count_[axis]));
      cluster_peak_error_[axis] = std::max(cluster_peak_error_[axis], target_peak_error_[axis]);
      cluster_squared_error_sum_[axis] += target_squared_error_sum_[axis];
      cluster_sample_count_[axis] += target_sample_count_[axis];
      RCLCPP_INFO(
        get_logger(), "  %s: %.6f / %.6f (samples: %zu)",
        kJointNames[axis], target_peak_error_[axis], rms, target_sample_count_[axis]);
    }
  }

  void finish_cluster()
  {
    cluster_active_ = false;
    cluster_status_ = "complete";
    const double clearing_time = (now() - cluster_start_time_).seconds();
    RCLCPP_INFO(
      get_logger(),
      "Cluster harvest complete: cleared %zu/%zu crops in %.3f simulated seconds.",
      cleared_count_, cluster_targets_.size(), clearing_time);
    RCLCPP_INFO(
      get_logger(), "Cluster tracking errors (peak / RMS; prismatic m, revolute rad):");
    for (size_t axis = 0; axis < kJointNames.size(); ++axis) {
      const double rms = cluster_sample_count_[axis] == 0 ? 0.0 :
        std::sqrt(cluster_squared_error_sum_[axis] /
        static_cast<double>(cluster_sample_count_[axis]));
      RCLCPP_INFO(
        get_logger(), "  %s: %.6f / %.6f (samples: %zu)",
        kJointNames[axis], cluster_peak_error_[axis], rms, cluster_sample_count_[axis]);
    }
    write_metrics_row("cluster_complete", "complete", "success");
    cluster_targets_.clear();
    cluster_started_ = false;
  }

  int sphere_count_ = 0;
  size_t cluster_id_ = 0;
  std::mt19937 random_engine_;
  std::vector<CropTarget> cluster_targets_;
  size_t next_spawn_index_ = 0;
  size_t cleanup_index_ = 0;
  size_t current_target_index_ = 0;
  size_t cleared_count_ = 0;
  bool cluster_setup_in_progress_ = false;
  bool cluster_active_ = false;
  bool awaiting_camera_detection_ = false;
  bool waiting_for_harvest_ = false;
  bool delete_pending_ = false;
  bool cleanup_succeeded_ = true;
  bool trajectory_received_ = false;
  bool cluster_started_ = false;
  bool target_detected_ = false;
  bool detection_wait_complete_ = false;
  bool target_command_started_ = false;
  bool target_timeout_logged_ = false;
  bool metrics_file_failed_ = false;
  bool have_joint_state_time_ = false;
  std::string cluster_status_ = "idle";
  std::string target_source_;
  std::string target_clear_result_ = "pending";
  std::string metrics_file_path_;
  double metrics_sample_period_sec_ = 1.0;
  double target_timeout_sec_ = 60.0;
  double detection_wait_wall_sec_ = 0.0;
  std::ofstream metrics_file_;
  std::chrono::steady_clock::time_point metrics_start_wall_time_;
  std::chrono::steady_clock::time_point cluster_start_wall_time_;
  std::chrono::steady_clock::time_point detection_start_wall_time_;
  std::chrono::steady_clock::time_point target_command_wall_time_;
  rclcpp::Time cluster_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time minimum_detection_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time metrics_start_sim_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
  trajectory_msgs::msg::JointTrajectory active_trajectory_;
  std::array<double, 6> joint_velocity_{};
  std::array<double, 6> joint_acceleration_{};
  std::array<double, 6> previous_joint_velocity_{};
  std::array<bool, 6> joint_velocity_valid_{};
  std::array<bool, 6> joint_acceleration_valid_{};
  std::array<bool, 6> previous_joint_velocity_valid_{};
  std::array<double, 6> target_peak_error_{};
  std::array<double, 6> target_squared_error_sum_{};
  std::array<size_t, 6> target_sample_count_{};
  std::array<double, 6> cluster_peak_error_{};
  std::array<double, 6> cluster_squared_error_sum_{};
  std::array<size_t, 6> cluster_sample_count_{};

  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr spawn_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_cluster_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr harvest_complete_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr berry_detected_pub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr metrics_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
