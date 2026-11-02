
**`harvest_ros2/README.md`**

```markdown
# harvest_ros2

This package contains ROS 2 nodes for HARVEST arm kinematics, trajectory generation, and effort control.

## Nodes

### `fk_node`

Subscribes to `/joint_states` (`sensor_msgs/msg/JointState`) and publishes the calculated end-effector pose on `/calculated_end_effector_pose` (`geometry_msgs/msg/PoseStamped`). Joint values are matched by name; the pose frame is `world`.

Run it with:

```bash
ros2 run harvest_ros2 fk_node
```

### `ik_node`

Subscribes to target poses on `/harvest_target_pose` (`geometry_msgs/msg/PoseStamped`) and current joint states on `/joint_states`. Target poses must use the `world` frame. For reachable targets, the node publishes joint targets on `/ik_node/raw_joint_targets` (`trajectory_msgs/msg/JointTrajectory`).

After the target joint positions are reached and settled, the node commands a 90-degree wrist-roll twist. It publishes `true` on `/harvest/cycle_complete` (`std_msgs/msg/Bool`) only after all six joints have settled following the twist. The node is started by `harvest_description`'s Gazebo launch file.

### `trajectory_planner_node`

Subscribes to raw joint targets on `/ik_node/raw_joint_targets` and feedback on `/joint_states`. It generates synchronized, jerk-limited seven-phase S-curve trajectories and publishes them on `/arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`).

### `gravity_compensation_node`

Subscribes to `/joint_states` and `/arm_controller/joint_trajectory`. It computes per-joint PID effort commands, including cumulative-angle gravity feed-forward for shoulder, elbow, and wrist pitch, and publishes them on `/effort_controller/commands` (`std_msgs/msg/Float64MultiArray`). Controller gains and effort limits are declared as ROS parameters in the node; the ros2_control YAML only configures the effort-command interface.

### `crop_simulator_node`

By default, creates a random cluster of 5-7 red, collidable strawberries in a 10 cm x 10 cm zone. It detects connected red components in the live `/camera/image_raw` stream and publishes the first selected crop name on `/crop_simulator/berry_detected` (`std_msgs/msg/String`) before sending its world pose to `/harvest_target_pose`. After `/harvest/cycle_complete`, it deletes that Gazebo entity and selects the next crop from its known simulated spawn positions; this avoids stalling when the end-effector camera is occluded at close range. Cluster completion logs total simulated clearing time and per-joint peak/RMS tracking error measured against the commanded trajectory; rail error is in metres and rotary-joint error is in radians. These errors quantify response to vibration or contact disturbances, but individual collision events are not separately counted.

The node truncates and recreates `harvest_evaluation_metrics.csv` at startup. It writes one telemetry row per second by default, including wall/simulation elapsed time, current status, crop detection and clearing results, target/cluster timing, and measured joint velocities and accelerations. Event rows record detection, target commands, clearing outcomes, Gazebo spawn/delete failures, and target timeouts. A target that exceeds the default 60-second wall-time limit is marked failed in the CSV, but sampling and simulation continue so a later recovery is still recorded. Velocity and acceleration columns are blank until feedback is available. The file is created in the launch working directory; configure another path or sampling interval with:

```bash
ros2 launch harvest_description gazebo.launch.py \
  metrics_file:=/absolute/path/to/harvest_evaluation_metrics.csv \
  metrics_sample_period_sec:=1.0 target_timeout_sec:=60.0
```

The Gazebo camera stream is bridged to `/camera/image_raw` (`sensor_msgs/msg/Image`). To view the live feed, install `ros-jazzy-rqt-image-view` if needed and run `ros2 run rqt_image_view rqt_image_view`, then select `/camera/image_raw` in the viewer. Use `ros2 topic echo /crop_simulator/berry_detected` to observe each visual detection.

Publish `std_msgs/msg/Empty` on `/crop_simulator/start_cluster` to start another cluster after the previous run. Set the `auto_start_cluster` parameter to `false` to disable startup spawning. The existing `/crop_simulator/spawn_strawberry` (`geometry_msgs/msg/PoseStamped`) topic remains available for single-crop spawning; its poses must use the `world` frame.

## Run with Gazebo

The recommended way to run the coordinated motion stack is to launch the robot simulation:

```bash
ros2 launch harvest_description gazebo.launch.py

```

This launch starts `ik_node`, `trajectory_planner_node`, `gravity_compensation_node`, and `crop_simulator_node`, along with the Gazebo robot and its ROS 2 controllers. To use the kinematics or control nodes independently, start the required nodes and provide the topics they subscribe to.

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths harvest_description harvest_ros2 --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Joint names

The nodes use these six joints, in this order for trajectory and effort commands:

1. `rail_joint`
2. `base_joint`
3. `shoulder_joint`
4. `elbow_joint`
5. `wrist_pitch_joint`
6. `wrist_roll_joint`

## License

See `LICENSE`.