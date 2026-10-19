import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
# ADD THIS IMPORT EXPLICITLY FOR JAZZY STRING PARSING
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('harvest_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # SDFormat resolves ROS package mesh URIs as model:// package resources.
    # Add the installed share directory so Gazebo can find this package's meshes.
    gazebo_resource_path = os.pathsep.join(filter(None, [
        os.path.dirname(pkg_share),
        os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ]))
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gazebo_resource_path)

    # 1. Create the LaunchConfiguration variable
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 2. Declare the argument so it can be modified via CLI (defaults to true for simulation)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

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
            '-name', 'berry_harvester',
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

    # Pass sim time to your Trajectory Planner
    trajectory_planner_node = Node(
        package='harvest_ros2',
        executable='trajectory_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Pass sim time to your IK Node
    ik_node = Node(
        package='harvest_ros2',
        executable='ik_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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
        set_gazebo_resource_path,
        declare_use_sim_time_cmd,  # This registers 'use_sim_time' so it exists
        robot_state_publisher_node,
        gazebo_sim,
        clock_bridge,
        spawn_robot,
        trajectory_planner_node,  # Added explicit launch execution
        ik_node,                  # Added explicit launch execution
        delay_broadcaster_after_spawn,
        delay_arm_controller_after_broadcaster
    ])
