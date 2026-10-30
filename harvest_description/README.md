
**`harvest_description/README.md`**

```markdown
# harvest_description

This package provides the HARVEST rail-mounted arm's robot description, meshes, RViz setup, and launch files for visualization and Gazebo simulation.

## Robot model

The Xacro model describes a rail carriage, a yawing turret, four arm actuators (shoulder pitch, two further pitch joints, and wrist roll), and a soft gripper. The three structural arm links are 150 mm center-to-center. Component masses are based on the actuator sheet: 200 g for the turret, 100 g for each of the first two links, 150 g for the wrist link, and 575 g per actuator. The rail carriage remains connected through `rail_joint`.

The model uses the supplied Onshape glTF assets for the base, pedestal, actuator housings, motors, and structural links, including the dark-blue distal link. These meshes provide the visuals; simple link collision bodies are used for Gazebo physics.

The base yaw pivot is 108 mm above the carriage, and the shoulder pivot is another 96 mm above it, offset 3.61 mm in X and -1.72 mm in Y to match the original assembly. A 20 mm tool-flange offset places the gripper on the fourth gearbox output face. With the arm upright at zero angle, the shoulder frame keeps it clear of the rail and floor. The tool-mount flange is perpendicular to the arm axis, so it is horizontal when the arm is upright.

Actuator-housing and output-pin visuals use the individual CAD output-drive frames. Each purple output pin is attached to the link it drives, so it follows that joint. The first two light structural links are twisted 90 degrees about their length to align their holes with the pin pattern. The dark-blue distal link remains untwisted, is aligned with the arm axis, and is positioned so its end ring mates with the fourth gearbox pin face.

The two light connector meshes use their full CAD scale so the small-end holes fit the output pins and the large-end holes clear the gearbox output faces. Their joint and collision dimensions are unchanged.

## Build

From the workspace root, source ROS 2 Jazzy, install dependencies, and build:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths harvest_description harvest_ros2 --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Display in RViz

Launch the model with the configured RViz view and joint-state sliders:

```bash
ros2 launch harvest_description display.launch.py
```

The launch file expands `urdf/robots/harvest_full.urdf.xacro`, runs `robot_state_publisher`, starts the joint-state publisher GUI, and opens RViz. You can override the model or RViz configuration:

```bash
ros2 launch harvest_description display.launch.py model:=/path/to/robot.urdf.xacro
ros2 launch harvest_description display.launch.py rviz_config:=/path/to/view.rviz
```

## Simulate in Gazebo

Launch Gazebo and the robot's ROS 2 control stack:

```bash
ros2 launch harvest_description gazebo.launch.py
```

The launch file starts Gazebo, bridges the simulation clock, spawns the robot, and starts the joint-state broadcaster and effort controller. It also starts the `harvest_ros2` inverse-kinematics, trajectory-planner, and gravity-compensation nodes. The simulation launch defaults `use_sim_time` to `true`; it can be overridden:

```bash
ros2 launch harvest_description gazebo.launch.py use_sim_time:=false
```

The simulation uses the controller configuration in `urdf/config/robot_controllers.yaml`.

## Repository layout

- `launch/` — RViz display and Gazebo simulation launch files.
- `meshes/` — CAD-derived visual assets.
- `rviz/` — RViz configuration.
- `urdf/mech/` — rail, arm, and gripper Xacro components.
- `urdf/robots/` — assembled robot Xacro model.
- `urdf/config/` — ROS 2 controller configuration.

## License

See `LICENSE`.