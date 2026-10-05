#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HarvesterController(Node):
    def __init__(self):
        super().__init__('harvester_controller')
        # Target the correct controller topic
        self.topic_name = '/arm_controller/joint_trajectory'
        # Create a publisher targeting the trajectory controller topic
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )

        # Check the network every 100ms for an active listener (gazebo)
        self.timer = self.create_timer(0.1, self.check_network_and_publish)
        self.published = False
        self.get_logger().info(f'Searching for Gazebo on "{self.topic_name}"...')
        # Wait a brief second for network discovery, then send the motion command
        # self.create_timer(1.0, self.send_movement_goal)

    def check_network_and_publish(self):
        # Count how many subscribers are listening to the topic
        num_subscribers = self.publisher_.get_subscription_count()

        if num_subscribers > 0:
            self.get_logger().info(f'Gazebo detected! Sending trajectory goal...')

            # Stop the timer so the goal is only sent once
            self.timer.cancel()

            # Construct the trajectory message and send it
            msg = JointTrajectory()
            msg.joint_names = [
                'rail_joint', 'base_joint', 'shoulder_joint', 
                'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint'
            ]

            point = JointTrajectoryPoint()
            # Test position targets
            point.positions = [1.0, 0.0, -1.2, 1.0, 0.8, 0.0]
            point.time_from_start = Duration(sec=6, nanosec=0) # Take 6 seconds

            msg.points = [point]
            self.publisher_.publish(msg)
            self.published = True

            # Give the network 5 seconds to clear buffers before shutting down cleanly
            self.create_timer(5.0, lambda: rclpy.shutdown())

        else:
            # Still waiting for network discovery
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HarvesterController()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass

if __name__ == '__main__':
    main()
