
**`harvest_ros2/README.md`**

```markdown
# harvest_ros2

This package contains ROS 2 nodes for HARVEST arm kinematics, trajectory generation, and effort control.

## Nodes

### `fk_node`

Subscribes to `/joint_states`, `/harvest_target_pose`, and `/arm_controller/joint_trajectory`. Publishes the measured tool-mount pose on `/calculated_end_effector_pose` and finger-midpoint grasp pose on `/calculated_grasp_point_pose` (both `geometry_msgs/msg/PoseStamped`, in the `world` frame). It logs commanded-endpoint FK error separately from measured tracking error; measured error is expected to be large while the arm is still moving. The grasp-point output includes the 45 mm finger offset and is the point compared with the berry target. Missing joint feedback is not silently treated as a zero position.

Run it with:

```bash
ros2 run harvest_ros2 fk_node
```

### `ik_node`

Subscribes to target poses on `/harvest_target_pose` (`geometry_msgs/msg/PoseStamped`) and current joint states on `/joint_states`. Target poses must use the `world` frame. For reachable targets, the node publishes joint targets on `/ik_node/raw_joint_targets` (`trajectory_msgs/msg/JointTrajectory`).

The node uses measured six-joint position and velocity feedback to detect settling, rather than relying on trajectory timing. Once the rail is within 15 mm, the rotary joints within 0.06 rad, and all joint speeds below 0.05 m/s or rad/s continuously for 200 ms, it commands a 90-degree wrist-roll twist. It publishes `true` on `/harvest/cycle_complete` (`std_msgs/msg/Bool`) only after the twist settles by the same criteria. Throttled progress logs identify joints that are still outside the limits. The node is started by `harvest_description`'s Gazebo launch file.

### `trajectory_planner_node`

Subscribes to raw joint targets on `/ik_node/raw_joint_targets` and feedback on `/joint_states`. It generates synchronized, jerk-limited seven-phase S-curve trajectories and publishes them on `/arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`).

### `gravity_compensation_node`

Subscribes to `/joint_states` and `/arm_controller/joint_trajectory`. It computes per-joint PID effort commands, including cumulative-angle gravity feed-forward for shoulder, elbow, and wrist pitch, and publishes them on `/effort_controller/commands` (`std_msgs/msg/Float64MultiArray`). Controller gains and effort limits are declared as ROS parameters in the node; the ros2_control YAML only configures the effort-command interface.

### `crop_simulator_node`

By default, creates a random cluster of 5-7 red, collidable strawberries in a 10 cm x 10 cm zone. It detects connected red components in the live `/camera/image_raw` stream and publishes the first selected crop name on `/crop_simulator/berry_detected` (`std_msgs/msg/String`). At selection it latches the simulator's known world coordinate and sends it once to `/harvest_target_pose`; this retained target remains valid after the berry leaves the camera view, avoiding duplicate IK commands. This is simulator ground-truth latching, not camera-derived 3D tracking. After `/harvest/cycle_complete`, it deletes that Gazebo entity and selects the next crop from its known simulated spawn positions; this avoids stalling when the end-effector camera is occluded at close range. Cluster completion logs total simulated clearing time and per-joint peak/RMS tracking error measured against the commanded trajectory; rail error is in metres and rotary-joint error is in radians. These errors quantify response to vibration or contact disturbances, but individual collision events are not separately counted.

The node truncates and recreates `harvest_evaluation_metrics.csv` at startup. It writes one telemetry row per second by default, including wall/simulation elapsed time, current status, crop detection and clearing results, target/cluster timing, and measured joint velocities and accelerations. Event rows record detection, target commands, clearing outcomes, Gazebo spawn/delete failures, and target timeouts. A target that exceeds the default 60-second wall-time limit is marked failed in the CSV, but sampling and simulation continue so a later recovery is still recorded. Velocity and acceleration columns are blank until feedback is available. The Gazebo launch defaults to saving the file in the source `harvest_ros2` package directory when that checkout is available, or under `~/.ros/harvest_ros2/` otherwise. The startup log prints the resolved path. Configure another path or sampling interval with:

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

This launch starts `ik_node`, `fk_node`, `trajectory_planner_node`, `gravity_compensation_node`, and `crop_simulator_node`, along with the Gazebo robot and its ROS 2 controllers. To use the kinematics or control nodes independently, start the required nodes and provide the topics they subscribe to.

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