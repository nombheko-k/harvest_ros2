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
    metrics_file = LaunchConfiguration('metrics_file')
    metrics_sample_period_sec = LaunchConfiguration('metrics_sample_period_sec')
    target_timeout_sec = LaunchConfiguration('target_timeout_sec')

    # 2. Declare the argument so it can be modified via CLI (defaults to true for simulation)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_metrics_file_cmd = DeclareLaunchArgument(
        'metrics_file',
        default_value='harvest_evaluation_metrics.csv',
        description='CSV evaluation metrics path; relative paths use the launch working directory'
    )
    declare_metrics_sample_period_cmd = DeclareLaunchArgument(
        'metrics_sample_period_sec',
        default_value='1.0',
        description='Wall-time interval between robot telemetry rows in the evaluation CSV'
    )
    declare_target_timeout_cmd = DeclareLaunchArgument(
        'target_timeout_sec',
        default_value='60.0',
        description='Wall-time limit for detection or clearing before a failure row is recorded'
    )

    xacro_file = os.path.join(pkg_share, 'urdf', 'robots', 'harvest_full.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'harvest.sdf')
    
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
        # Use the project world because Gazebo's built-in empty world does not load Sensors.
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Bridge the simulation clock, camera feed, and Gazebo entity services to ROS.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/world/empty/create@ros_gz_interfaces/srv/SpawnEntity',
                   '/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity'],
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

    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller'],
        output='screen',
    )

    # Pass sim time to your Trajectory Planner
    trajectory_planner_node = Node(
        package='harvest_ros2',
        executable='trajectory_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gravity_compensation_node = Node(
    package='harvest_ros2',
    executable='gravity_compensation_node',
    output='screen',
    parameters=[{
        # Approximate each pitch joint's gravity coefficient from downstream URDF masses and lever arms.
        'shoulder.max_gravity': 6.95,
        'elbow.max_gravity': 3.80,
        'wrist_p.max_gravity': 1.63,
        'shoulder.limit': 30.0,
        'use_sim_time': use_sim_time
    }]
    )

    # Pass sim time to your IK Node
    ik_node = Node(
        package='harvest_ros2',
        executable='ik_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Keep the crop topic subscriber running with the same Gazebo service bridge.
    crop_simulator_node = Node(
        package='harvest_ros2',
        executable='crop_simulator_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'metrics_file': metrics_file,
            'metrics_sample_period_sec': ParameterValue(
                metrics_sample_period_sec, value_type=float),
            'target_timeout_sec': ParameterValue(target_timeout_sec, value_type=float)
        }]
    )

    # Delayed event hooks: Sequential startup orchestration
    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_effort_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[effort_controller_spawner],
        )
    )

    return LaunchDescription([
        set_gazebo_resource_path,
        declare_use_sim_time_cmd,  # This registers 'use_sim_time' so it exists
        declare_metrics_file_cmd,
        declare_metrics_sample_period_cmd,
        declare_target_timeout_cmd,
        robot_state_publisher_node,
        gazebo_sim,
        clock_bridge,
        spawn_robot,
        trajectory_planner_node,  # Added explicit launch execution
        gravity_compensation_node,
        ik_node,                  # Added explicit launch execution
        crop_simulator_node,
        delay_broadcaster_after_spawn,
        delay_effort_controller_after_broadcaster
    ])
