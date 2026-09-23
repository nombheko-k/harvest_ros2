import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
# ADD THIS IMPORT EXPLICITLY FOR JAZZY STRING PARSING
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('harvest_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_share, 'urdf', 'robots', 'harvest_full.urdf.xacro')
    
    # WRAP THE COMMAND STRING SO LAUNCH DOES NOT CONFUSE XML FOR YAML
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]), 
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Core Gazebo Sim framework
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Clock bridge to sync physics engine and controller_manager
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawn entity node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'raspberry_harvester',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ]
    )

    # ros2_control managers activation
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    # Delayed event hooks: Sequential startup orchestration
    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_sim,
        clock_bridge,
        spawn_robot,
        delay_broadcaster_after_spawn,
        delay_arm_controller_after_broadcaster
    ])
