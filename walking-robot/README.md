# Stepper Motor Control System

This project provides ROS2 packages for controlling stepper and servo motors.

## Directory Structure

### `controller/`
- Contains the ROS2 package for controller functionality
- **Package Name**: `stepper_motor`
- **Purpose**: Controls the motor functionality including movement and servo control.
- **Features**:
  - Serial communication with the hardware via /dev/ttyACM0
  - ROS2 topic communication via `stepper_motor_command`
  - Direct GPIO control for stepper motor operations

### `input/`
- Contains the ROS2 package for receiving and processing input commands
- **Package Name**: `stepper_motor`
- **Purpose**: Provides tools for input command processing and GUI control.
- **Features**:
  - GUI for controlling servo angles and stepper motor movement
  - Real-time robot visualization and manipulation
  - Recording and playback capability

## Building and Running

### For controller:
```bash
cd controller
colcon build
source install/setup.bash
ros2 run stepper_motor controller_node
```

### For input:
```bash
cd input
colcon build
source install/setup.bash
ros2 run stepper_motor input_node
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
- Additional Python libraries for GUI
  - Tkinter
  - pyserial (for controller communication)

## Components Detail

### Controller Package (`controller/src/stepper_motor/`)

#### core Files:
- `controller_node.py`: Serial communication node for forwarding commands to the hardware and handling responses via the serial port
- `stepper_motor.py`: Provides direct GPIO control over stepper motors

#### Features:
- Listens on `stepper_motor_command` topic and forwards commands via a serial connection
- Implements serial error handling and logging
- Commands hardware to rotate motors and control servo angles using received commands

### Input Package (`input/src/stepper_motor/`)

#### core Files:
- `input_node.py`: GUI for user command input and visualization with advanced features
- `controller_node.py`: Alternative controller node for direct GPIO interactions

#### GUI Features:
- **4-Leg Robot**: Control of 4 legs with individual servo angles
- **Visualization**: Real-time representation of the robot's position and orientation
- **Interactive Manipulation**: Use mouse to adjust servo positions
- **Movements**: Predefined grip and walk animations
- **Playback and Recording**: Capture and replay sequences of movements

## Communication Protocol

### ROS2 Topics:
- **Topic Name**: `stepper_motor_command`
- **Message Type**: `std_msgs/String`
- **Direction**: input_node → controller_node

### Command Formats:
- `L`: Rotate stepper motor left
- `R`: Rotate stepper motor right
- `S1:90`: Set servo 1 to 90 degrees
- `A 90;B 45;C 120;...`: Multi-servo command format (GUI mode)

## GPIO Pin Mapping

### Stepper Motor Pins:
- **GPIO 17**: Pin 1
- **GPIO 18**: Pin 2
- **GPIO 27**: Pin 3
- **GPIO 22**: Pin 4

### Servo Motors:
- **GPIO 4**: Servo 1
- **GPIO 5**: Servo 2
- **GPIO 6**: Servo 3
- **GPIO 13**: Servo 4

## Usage Instructions

### Basic Setup Steps:
1. **Hardware Setup**: Connect motors to GPIO pins on the Raspberry Pi
2. **Serial Configuration**: Make sure `/dev/ttyACM0` is accessible for serial communications
3. **User Permissions**:
   ```bash
   sudo usermod -a -G dialout $USER
   ```

### Execution:
1. **Controller Node Execution**:
   ```bash
   cd controller
   colcon build
   source install/setup.bash
   ros2 run stepper_motor controller_node
   ```

2. **GUI Node Execution**:
   ```bash
   cd input
   colcon build
   source install/setup.bash
   ros2 run stepper_motor input_node
   ```

### Interactive GUI:
- **Sliders**: Move sliders to adjust the servo angles
- **Grip Command**: Activate predefined grip sequence
- **Walking Sequence**: Start or stop the walking animation
- **Record Movements**: Press record to save motion sequences
- **Manipulate Joints**: Click and drag to reposition joints in real-time

## Troubleshooting Tips

### Common Pitfalls:
1. **Serial Port Unavailable**:
   - Check the existence of `/dev/ttyACM0`
   - Ensure correct user permissions
   - Try alternative USB ports if necessary

2. **GPIO Access Errors**:
   - Execute with `sudo` if needed
   - Verify that other applications are not using the same GPIO resources

3. **ROS2 Issues**:
   - Confirm both nodes are operational
   - Use `ros2 topic list` to verify topic activity
   - Echo messages with `ros2 topic echo /stepper_motor_command` to monitor traffic

4. **GUI Display Errors**:
   - Confirm X11 forwarding for remote access
   - Ensure Tkinter is installed and operational
   - Validate display environment configurations

### Quick Testing Commands:
```bash
# Publish Test Command
ros2 topic pub /stepper_motor_command std_msgs/String "data: 'L'"

# Nodes List
ros2 node list

# Node Information
ros2 node info /controller_node
ros2 node info /stepper_motor_control_gui
```

## Technical Specifications

### Stepper Motor Control:
- **Step Sequence**: 8-step sequence for precise motor control
- **Step Timing**: 0.001 seconds between steps
- **Default Steps**: 512 steps per rotation command
- **Directions**: Left (forward sequence) and Right (reverse sequence)
- **GPIO Mode**: BCM pin numbering

### Servo Motor Control:
- **PWM Frequency**: 50Hz for standard servo operation
- **Duty Cycle Calculation**: `duty = angle / 18.0 + 2.5`
- **Angle Range**: 0-180 degrees
- **Servo Mapping** (GUI to Hardware):
  - Leg 1 Upper: A → Some servos are inverted (180 - angle)
  - Leg 1 Lower: B → Direct mapping
  - Leg 2 Upper: E → Direct mapping
  - Leg 2 Lower: F → Inverted (180 - angle)
  - Leg 3 Upper: C → Inverted (180 - angle)
  - Leg 3 Lower: D → Direct mapping
  - Leg 4 Upper: H → Inverted (180 - angle)
  - Leg 4 Lower: G → Inverted (180 - angle)

### GUI Technical Details:
- **Canvas Size**: 1000x400 pixels
- **Robot Leg Positions**: 
  - Leg 1: x=150, Leg 2: x=350, Leg 3: x=650, Leg 4: x=800
  - Base height: y=300 (ground level)
  - Robot body: y=200 (100 pixels above ground)
- **Leg Segment Length**: 80 pixels each
- **Animation Frame Rate**: 60 FPS (16ms intervals)
- **Recording Frame Rate**: 30 FPS (33ms intervals)
- **Command Throttling**: 0.1 seconds between published commands

### Walking Animation Keyframes:
1. **Initial Position**: All legs in neutral stance
2. **Preparation**: Legs 3&4 lift for weight shift
3. **Sway**: Body weight shifts to legs 1&2
4. **Step**: Legs 1&2 adjust position
5. **Recovery**: Legs move to new position

### Serial Communication:
- **Port**: `/dev/ttyACM0` (configurable)
- **Baud Rate**: 9600
- **Timeout**: 1 second
- **Protocol**: Command + newline, optional response reading
- **Error Handling**: Try-catch with logging

### Command Processing:
- **Simple Commands**: 'L', 'R' for stepper motor
- **Servo Commands**: 'S1:90' format (servo number : angle)
- **Multi-Servo**: 'A 90;B 45;C 120;...' format from GUI
- **Input Validation**: Angle range checking (0-180 degrees)

### Memory and Performance:
- **Recording Storage**: In-memory list of timestamp-angle pairs
- **Animation Interpolation**: Linear interpolation between keyframes
- **Thread Safety**: GUI runs in main thread, ROS2 in separate daemon thread
- **Motor Disable**: Special value 999 used to disable motors during animation

## Available Entry Points

### Controller Package:
- `controller_node`: Main serial communication node
- `input_node`: Simple CLI input interface

### Input Package:
- `input_node`: Advanced GUI with visualization
- `controller_node`: Direct GPIO control alternative

## Additional Notes

- Both packages use identical naming (`stepper_motor`) for consistency
- The GUI provides both visual control and direct GPIO manipulation options
- Serial communication centralizes hardware abstraction in the controller
- The visualization system includes real-time joint calculations and inverse kinematics
- Animation system supports both predefined sequences and user-recorded motions
- Debug mode available for showing angle calculations and joint vectors

