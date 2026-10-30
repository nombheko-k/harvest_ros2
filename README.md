# HARVEST ROS 2

![Ubuntu 24.04](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS 2 Jazzy](https://img.shields.io/ros/v/jazzy/rclcpp)

ROS 2 packages for the HARVEST rail-mounted arm. The repository contains the robot description and simulation launch files in `harvest_description`, and kinematics and motion-control nodes in `harvest_ros2`.

## Packages

- **`harvest_description`** — robot URDF/Xacro model, meshes, RViz configuration, and display and Gazebo launch files. See [`harvest_description/README.md`](harvest_description/README.md).
- **`harvest_ros2`** — forward and inverse kinematics, trajectory planning, and gravity-compensation nodes. See [`harvest_ros2/README.md`](harvest_ros2/README.md).

## Requirements

The packages target **ROS 2 Jazzy** and Ubuntu 24.04. The description and simulation launches also depend on the ROS 2, RViz, Xacro, and Gazebo packages declared in `harvest_description/package.xml`.

## Build

From the repository root, source ROS 2 and install the package dependencies:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths harvest_description harvest_ros2 --ignore-src -r -y
```

Build both packages with `colcon`:

```bash
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` in each new terminal where you want to use the packages

## View the robot

Launch the robot description with RViz and the joint-state sliders:

```bash
ros2 launch harvest_description display.launch.py
```

## Run the Gazebo simulation

Launch Gazebo with the robot, controllers, inverse kinematics, trajectory planner, and gravity-compensation node:

```bash
ros2 launch harvest_description gazebo.launch.py
```

The inverse kinematics node accepts target poses on `/harvest_target_pose`. Its target frame must be `world`. Joint positions are reported on `/joint_states`.

## Nodes and topics

| Node | Subscribes to | Publishes to | Purpose |
| --- | --- | --- | --- |
| `fk_node` | `/joint_states` | `/calculated_end_effector_pose` | Computes the end-effector pose from the named joint positions. |
| `ik_node` | `/harvest_target_pose`, `/joint_states` | `/ik_node/raw_joint_targets` | Computes joint targets for a pose and commands a wrist-roll twist after reaching the target. |
| `trajectory_planner_node` | `/ik_node/raw_joint_targets`, `/joint_states` | `/arm_controller/joint_trajectory` | Converts joint targets into synchronized, jerk-limited trajectories. |
| `gravity_compensation_node` | `/joint_states`, `/arm_controller/joint_trajectory` | `/effort_controller/commands` | Applies per-joint PID effort control and shoulder gravity compensation. |

Run the forward-kinematics node separately when needed:

```bash
ros2 run harvest_ros2 fk_node
```

The Gazebo launch starts the inverse-kinematics, trajectory-planner, and gravity-compensation nodes; it does not start `fk_node`.

## License

The packages declare the BSD 3-Clause license. See each package's `LICENSE` file.