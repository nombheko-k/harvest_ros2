#include <memory>
#include <string>
#include <sstream>
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

        RCLCPP_INFO(this->get_logger(), "Strawberry Crop Object Simulation Node Online (Listening on Topic).");
    }

private:
    // The callback now accepts a standard message SharedPtr
    void handle_spawn_msg(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        if (!spawn_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Gazebo entity spawner service (/spawn_entity) not available!");
            return;
        }

        std::string name = "strawberry_" + std::to_string(sphere_count_++);
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;

        // Generate a 20mm visual sphere with minimal physics mass properties
        std::stringstream sdf;
        sdf << "<sdf version='1.7'>"
            << "  <model name='" << name << "'>"
            << "    <pose>" << x << " " << y << " " << z << " 0 0 0</pose>"
            << "    <link name='link'>"
            << "      <inertial>"
            << "        <mass>0.005</mass>" // 5 grams
            << "        <inertia><ixx>8e-7</ixx><ixy>0</ixy><ixz>0</ixz><iyy>8e-7</iyy><iyz>0</iyz><izz>8e-7</izz></inertia>"
            << "      </inertial>"
            << "      <visual name='visual'>"
            << "        <geometry><sphere><radius>0.01</radius></sphere></geometry>" // 1cm radius
            << "        <material>"
            << "          <ambient>0.8 0.0 0.1 1.0</ambient>" // Crimson Red
            << "          <diffuse>0.8 0.0 0.1 1.0</diffuse>"
            << "        </material>"
            << "      </visual>"
            << "      <collision name='collision'>"
            << "        <geometry><sphere><radius>0.01</radius></sphere></geometry>"
            << "      </collision>"
            << "    </link>"
            << "  </model>"
            << "</sdf>";

        auto srv_req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
        srv_req->entity_factory.sdf = sdf.str();
        srv_req->entity_factory.name = name;

        RCLCPP_INFO(this->get_logger(), "Spawning crop object '%s' at: (X: %.3f, Y: %.3f, Z: %.3f)", name.c_str(), x, y, z);
        
        auto future = spawn_client_->async_send_request(srv_req);
    }

    int sphere_count_;
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr spawn_sub_; 
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CropSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
