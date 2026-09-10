import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve files from the installed package so this launch file also works
    # when started outside the source workspace.
    pkg_share = get_package_share_directory('harvest_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'robots', 'harvest_280.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'view_robot.rviz')

    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # Expand the Xacro at launch time and publish the resulting URDF.
            'robot_description': Command(['xacro', ' ', model]),
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file',
        ),
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file',
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])

# End of the RViz display launch description.
