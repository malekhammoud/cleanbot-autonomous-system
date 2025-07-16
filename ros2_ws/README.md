# ROS2 Workspace

This directory contains the ROS2 workspace for the robot control system.

## Structure

- `src/robot/` - ROS2 package for robot control
  - `robot/controller_node.py` - Main controller node for stepper and servo motors
  - `robot/input_node.py` - Input handling node
  - `robot/robot.py` - Basic robot functionality

## Building

To build the workspace:

```bash
cd ros2_ws
colcon build
```

## Running

Source the workspace:
```bash
source install/setup.bash
```

Run the controller node:
```bash
ros2 run robot controller_node
```

Run the input node:
```bash
ros2 run robot input_node
```

## Dependencies

- ROS2 (tested with Humble)
- RPi.GPIO (for Raspberry Pi GPIO control)
- Python 3
