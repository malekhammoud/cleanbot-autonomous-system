# Walking Robot Control System

This directory contains ROS2 workspaces for controlling walking robots.

## Directory Structure

### `ros2_ws_original/`
- Contains the original robot control package
- **Package Name**: `robot`
- **Purpose**: Basic stepper motor and servo control
- **Features**:
  - Stepper motor control for movement
  - Servo motor control for manipulation
  - ROS2 topic communication via `robot_command`

### `ros2_ws_robot/`
- Contains the enhanced robot walking package
- **Package Name**: `robot_walking`
- **Purpose**: Advanced walking robot control
- **Features**:
  - Walking gait control
  - Enhanced motor coordination
  - Advanced sensor integration

## Building and Running

### For ros2_ws_original:
```bash
cd ros2_ws_original
colcon build
source install/setup.bash
ros2 run robot controller_node
ros2 run robot input_node
```

### For ros2_ws_robot:
```bash
cd ros2_ws_robot
colcon build
source install/setup.bash
ros2 run robot_walking controller_node
ros2 run robot_walking input_node
```

## Hardware Requirements

- Raspberry Pi (4B recommended)
- Stepper motors
- Servo motors
- GPIO breakout board
- Power supply suitable for motors

## Dependencies

- ROS2 Humble
- RPi.GPIO
- Python 3.8+
