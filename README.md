# litter-automation
A Science Fair project for automated litter detection and collection

## Project Structure

- **Main Python scripts**: Drone control, image processing, and automation logic
- **ros2_ws/**: ROS2 workspace containing robot control system
  - Motor control nodes for stepper motors and servos
  - Input handling and command processing
  - Hardware interface for Raspberry Pi GPIO

## Components

### Drone System
- Autonomous flight control
- GPS-based navigation
- Camera stream processing
- Litter detection using computer vision

### Robot Control System (ROS2)
- Stepper motor control for movement
- Servo motor control for manipulation
- ROS2 node communication
- Hardware abstraction layer

## Setup

### For Drone System
Run the main Python scripts directly

### For Robot System
See `ros2_ws/README.md` for ROS2 workspace setup instructions
