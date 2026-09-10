import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve package resources from the install space, not from a hard-coded
    # source path, so this launch file works after a normal colcon build.
    pkg_share = get_package_share_directory('harvest_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_model_path = os.path.join(
        pkg_share, 'urdf', 'robots', 'harvest_280.urdf.xacro'
    )
    model = LaunchConfiguration('model')
    use_control = LaunchConfiguration('use_control')
    # Pass the control switch through to the robot Xacro. Control is opt-in
    # because the visual Gazebo launch must also work without controllers.
    robot_description_content = Command([
        'xacro', ' ', model, ' use_gz_control:=', use_control
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # Gazebo publishes the simulation clock on /clock.
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }]
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Start an empty running Gazebo Sim world.
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            # Spawn from robot_description after robot_state_publisher starts.
            '-name', 'raspberry_harvester',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        # Do not wait for controller_manager during a visual-only launch.
        condition=IfCondition(use_control),
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        condition=IfCondition(use_control),
        output='screen',
    )

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
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to the robot Xacro file',
        ),
        DeclareLaunchArgument(
            name='use_control',
            default_value='false',
            description='Enable the Gazebo ros2_control plugin and controllers',
        ),
        robot_state_publisher_node,
        gazebo_sim,
        spawn_robot,
        delay_broadcaster_after_spawn,
        delay_arm_controller_after_broadcaster,
    ])
