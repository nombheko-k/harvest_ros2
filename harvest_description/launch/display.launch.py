import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
# ADD THIS IMPORT EXPLICITLY FOR JAZZY STRING PARSING
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('harvest_description')
    # Paths to files
    default_model_path = os.path.join(pkg_share, 'urdf', 'robots', 'harvest_full.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'view_robot.rviz')

    # Launch configurations
    model = LaunchConfiguration('model', default=default_model_path)
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz_config_path)

    # WRAP THE COMMAND STRING SO LAUNCH DOES NOT CONFUSE XML FOR YAML
    robot_description_content = ParameterValue(
        Command(['xacro ', model]), 
        value_type=str
    )

    # Robot State Publisher Node (processes Xacro using Command substitution)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Joint State Publisher GUI Node (provides sliders to move the joints in RViz)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
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
