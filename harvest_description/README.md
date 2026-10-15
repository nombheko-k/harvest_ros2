# harvest_description

This package describes the HARVEST rail-mounted arm for RViz and Gazebo.

## Arm model

The arm xacro follows the actuator sheet: a yawing turret and four arm actuators
(shoulder pitch, two further pitch joints, and wrist roll). The three structural
links are 150 mm center-to-center. Component masses are taken from the sheet:
200 g for the turret, 100 g for each of the first two links, 150 g for the wrist
link, and 575 g per actuator. The rail carriage remains attached through
`rail_joint`.

The arm uses the supplied Onshape glTF assets for the base, pedestal, actuator
housings, motors, and structural links, including the dark-blue distal link.
These are installed with the package and used for visuals; simple link collision
bodies are retained for robust Gazebo physics. The base yaw pivot is 108 mm above
the carriage, and the shoulder pivot is another 96 mm above it. A 20 mm tool
flange offset places the gripper on the fourth gearbox output face. The shoulder
frame keeps the zero-angle arm straight up, clear of the rail and floor.

Actuator housing and output-pin visuals use the individual CAD output-drive
frames. Each purple output pin is attached to the link it drives, so it follows
that joint; the first two light structural links are twisted 90 degrees about
their length to align their holes with the pin pattern. The dark-blue distal
link remains untwisted, is aligned exactly with the arm axis, and is positioned
so its end ring mates with the fourth gearbox pin face.
The two light connector meshes use their full CAD scale so the small-end holes
fit the output pins and the large-end holes clear the gearbox output faces;
their joint and collision dimensions are unchanged.

## Launch

After building and sourcing the ROS 2 workspace:

```bash
ros2 launch harvest_description display.launch.py
ros2 launch harvest_description gazebo.launch.py
```

The display launch publishes the xacro-expanded robot description to RViz.
The Gazebo launch spawns the same model with the configured ros2_control
interfaces. The FK and IK nodes in `harvest_ros2` use the same 150 mm link lengths,
229 mm shoulder-pivot height from the rail origin, 20 mm tool-flange offset,
startup pose, and joint names as this description.
