
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

After the target joint positions are reached, the node commands a 90-degree wrist-roll twist. The node is started by `harvest_description`'s Gazebo launch file.

### `trajectory_planner_node`

Subscribes to raw joint targets on `/ik_node/raw_joint_targets` and feedback on `/joint_states`. It generates synchronized, jerk-limited seven-phase S-curve trajectories and publishes them on `/arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`).

### `gravity_compensation_node`

Subscribes to `/joint_states` and `/arm_controller/joint_trajectory`. It computes per-joint PID effort commands, including shoulder gravity compensation, and publishes them on `/effort_controller/commands` (`std_msgs/msg/Float64MultiArray`). Controller gains and effort limits are declared as ROS parameters in the node.

## Run with Gazebo

The recommended way to run the coordinated motion stack is to launch the robot simulation:

```bash
ros2 launch harvest_description gazebo.launch.py

```

This launch starts `ik_node`, `trajectory_planner_node`, and `gravity_compensation_node`, along with the Gazebo robot and its ROS 2 controllers. To use the kinematics or control nodes independently, start the required nodes and provide the topics they subscribe to.

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