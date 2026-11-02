#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "ros_gz_interfaces/srv/delete_entity.hpp"

class CropSimulatorNode : public rclcpp::Node {
public:
    CropSimulatorNode() : Node("crop_simulator_node"), sphere_count_(0) {
        // Use set_parameter instead of declare_parameter to avoid collision
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // Clients to interface with the Gazebo simulation plugins
        spawn_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/empty/create");
        delete_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/empty/remove");

        // Changed from a service to a standard Topic Subscriber
        spawn_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/crop_simulator/spawn_strawberry", 10,
            std::bind(&CropSimulatorNode::handle_spawn_msg, this, std::placeholders::_1));

        // Use the detected crop pose as the arm target after Gazebo confirms the spawn.
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/harvest_target_pose", 10);

        RCLCPP_INFO(this->get_logger(), "Strawberry Crop Object Simulation Node Online (Listening on Topic).");
    }

private:
    // The callback now accepts a standard message SharedPtr
    void handle_spawn_msg(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        if (!spawn_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Gazebo entity spawner service (/world/empty/create) not available; "
                "check that gazebo.launch.py started the ROS-Gazebo service bridge.");
            return;
        }

        std::string name = "strawberry_" + std::to_string(sphere_count_++);
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;

        // Keep the crop stationary and large enough to see clearly in the Gazebo view.
        std::stringstream sdf;
        sdf << "<sdf version='1.7'>"
            << "  <model name='" << name << "'>"
            << "    <static>true</static>"
            << "    <link name='link'>"
            << "      <visual name='visual'>"
            << "        <geometry><sphere><radius>0.025</radius></sphere></geometry>"
            << "        <material>"
            << "          <ambient>1.0 0.0 0.05 1.0</ambient>"
            << "          <diffuse>1.0 0.0 0.05 1.0</diffuse>"
            << "          <emissive>0.25 0.0 0.01 1.0</emissive>"
            << "        </material>"
            << "      </visual>"
            << "      <collision name='collision'>"
            << "        <geometry><sphere><radius>0.025</radius></sphere></geometry>"
            << "      </collision>"
            << "    </link>"
            << "  </model>"
            << "</sdf>";

        auto srv_req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
        srv_req->entity_factory.sdf = sdf.str();
        srv_req->entity_factory.name = name;
        // Set the factory pose explicitly; a sphere needs identity orientation even if the input quaternion is unset.
        srv_req->entity_factory.pose.position = msg->pose.position;
        srv_req->entity_factory.pose.orientation.w = 1.0;
        srv_req->entity_factory.relative_to = "world";

        RCLCPP_INFO(this->get_logger(), "Spawning crop object '%s' at: (X: %.3f, Y: %.3f, Z: %.3f)", name.c_str(), x, y, z);
        
        // Check the bridged service response; sending the request alone does not confirm a spawn.
        spawn_client_->async_send_request(
            srv_req,
            [this, name, msg](rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future) {
                const auto response = future.get();
                if (!response->success) {
                    RCLCPP_ERROR(
                        this->get_logger(), "Gazebo failed to spawn crop object '%s'.", name.c_str());
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Gazebo spawned crop object '%s'.", name.c_str());

                if (msg->header.frame_id != "world") {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Spawned crop, but cannot command IK: target frame is '%s', expected 'world'.",
                        msg->header.frame_id.c_str());
                    return;
                }
                target_pub_->publish(*msg);
                RCLCPP_INFO(this->get_logger(), "Sent strawberry pose to IK on /harvest_target_pose.");
            });
    }

    int sphere_count_;
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr spawn_sub_; 
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CropSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
