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

The inverse kinematics node accepts target poses on `/harvest_target_pose`; target frames must be `world`. By default, `crop_simulator_node` spawns a random cluster of 5-7 red strawberries in a 10 cm x 10 cm zone. Berries are visual-only: their spherical collision geometry would prevent the gripper from reaching the berry-center grasp target. The bridged camera stream on `/camera/image_raw` is used to detect red berries; each detection is published as the crop name on `/crop_simulator/berry_detected`, then that berry is harvested. After all six joints settle following the wrist twist, the simulator deletes the crop and waits for a fresh camera detection before proceeding. Publish `std_msgs/msg/Empty` on `/crop_simulator/start_cluster` to start another cluster, or set `auto_start_cluster:=false` to disable startup spawning. The legacy `/crop_simulator/spawn_strawberry` topic remains available for single crops. Joint positions are reported on `/joint_states`.

To view the live camera feed, run `ros2 run rqt_image_view rqt_image_view` and select `/camera/image_raw`. Install `ros-jazzy-rqt-image-view` if the viewer is not already available.

## Nodes and topics

| Node | Subscribes to | Publishes to | Purpose |
| --- | --- | --- | --- |
| `fk_node` | `/joint_states`, `/harvest_target_pose`, `/arm_controller/joint_trajectory` | `/calculated_end_effector_pose`, `/calculated_grasp_point_pose` | Computes measured FK and reports commanded-endpoint IK/FK error separately from in-flight tracking error. |
| `ik_node` | `/harvest_target_pose`, `/joint_states` | `/ik_node/raw_joint_targets`, `/harvest/cycle_complete` | Computes joint targets, commands a wrist-roll twist, then signals completion after six-joint settling. |
| `trajectory_planner_node` | `/ik_node/raw_joint_targets`, `/joint_states` | `/arm_controller/joint_trajectory` | Converts joint targets into synchronized, jerk-limited trajectories. |
| `gravity_compensation_node` | `/joint_states`, `/arm_controller/joint_trajectory` | `/effort_controller/commands` | Applies per-joint PID effort control and shoulder gravity compensation. |
| `crop_simulator_node` | `/crop_simulator/spawn_strawberry`, `/crop_simulator/start_cluster`, `/camera/image_raw`, `/harvest/cycle_complete`, `/joint_states`, `/arm_controller/joint_trajectory` | `/harvest_target_pose`, `/crop_simulator/berry_detected`, Gazebo `/world/empty/create`, `/world/empty/remove` | Detects red berries from the camera stream, sequences harvesting, deletes harvested crops, and reports clearing time and per-axis tracking errors. |

PID defaults are declared by `gravity_compensation_node`; the ros2_control YAML configures the effort-command interface and does not tune this custom PID loop.

Run the forward-kinematics node separately when needed:

```bash
ros2 run harvest_ros2 fk_node
```

The Gazebo launch also starts `fk_node` to publish the measured tool and grasp-point poses and report endpoint FK checks.

## License

The packages declare the BSD 3-Clause license. See each package's `LICENSE` file.