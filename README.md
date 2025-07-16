# 🚁🤖 Litter Automation System

A comprehensive Science Fair project for automated litter detection and collection using drones and walking robots.

## 📁 Project Structure

```
litter-automation/
├── drone-system/                    # 🚁 Complete drone automation system
│   ├── scripts/                     # 📝 Python flight control scripts (17 files)
│   │   ├── main.py                 # Main automation control
│   │   ├── drone.py                # Drone control class
│   │   ├── control.py              # Flight control logic
│   │   ├── gps-camera-record.py    # GPS-enabled recording
│   │   ├── camera-segmentation.py  # Image processing
│   │   ├── live-detect.py          # Real-time detection
│   ├── data/                       # 📊 Organized mission data
│   ├── config/                     # ⚙️ MAVLink configuration
│   ├── logs/                       # 📋 System logs
├── walking-robot/                  # 🤖 Ground-based robot control
│   ├── ros2_ws_original/           # 🔧 Original robot package
│   ├── ros2_ws_robot/              # 🆕 Enhanced walking robot
├── .gitignore                      # Git ignore configuration
└── README.md                       # This file
```

## 🛠 Project Overview

This project aims to tackle environmental issues by automating the detection and collection of litter using drones and walking robots. The drone system is equipped with computer vision capabilities for real-time litter detection during flight, while the walking robot is designed for advanced locomotion to handle uneven terrains and collect litter.

## 🚁 Drone System

- **Autonomous Flight Control**: GPS-based navigation and waypoint following
- **Computer Vision**: Real-time litter detection using OpenCV
- **Data Collection**: Synchronized GPS, video, and image capture
- **Mission Planning**: Automated survey patterns and area coverage
- **MAVLink Integration**: Direct drone communication and control

## 🤖 Walking Robot System

- **Dual ROS2 Workspaces**: Original and enhanced robot control
- **Motor Control**: Stepper motor and servo coordination
- **Sensor Integration**: GPIO-based hardware interface
- **Walking Gaits**: Advanced locomotion algorithms
- **Topic Communication**: ROS2 node messaging system

## 🚀 Quick Start

### Drone System
```bash
cd drone-system/scripts
python main.py              # Start full automation
python takeoff_and_land.py  # Test basic flight
```

### Walking Robot
```bash
cd walking-robot/ros2_ws_original
colcon build && source install/setup.bash
ros2 run robot controller_node

cd walking-robot/ros2_ws_robot
colcon build && source install/setup.bash
ros2 run robot_walking controller_node
```

## 📋 Hardware Requirements

### Drone Setup
- MAVLink-compatible drone (ArduPilot/PX4)
- Companion computer (Raspberry Pi 4B+)
- Camera module with gimbal

### Robot Setup
- Raspberry Pi 4B
- Stepper motors for locomotion
- Servo motors for manipulation

## 🔧 Dependencies

### Technologies Used

- **Dronekit-Python**: For drone flight control and mission management
- **RPi.GPIO**: For Raspberry Pi GPIO control
- **OpenCV**: For real-time image processing and litter detection
- **pymavlink**: For MAVLink communication protocol
- **ROS2 Humble**: For robot control and messaging

```bash
pip install opencv-python mavlink numpy sqlite3 rpi.gpio
sudo apt install ros-humble-desktop
```

## 📊 Features

- ✅ **Autonomous Navigation**: GPS-based flight planning
- ✅ **Real-time Detection**: Computer vision litter identification
- ✅ **Data Logging**: Comprehensive mission data storage
- ✅ **Multi-platform**: Drone and ground robot coordination

## 📚 Documentation

- [`drone-system/README.md`](drone-system/README.md) - Detailed drone setup
- [`walking-robot/README.md`](walking-robot/README.md) - Robot control guide

## 🏆 Science Fair Impact

This project demonstrates:
- **Environmental Awareness**: Automated litter detection and mapping
- **Robotics Integration**: Multi-platform autonomous systems
- **Data Science**: GPS tracking and analysis

## 📖 Related Resources

### Articles and Documentation:
- **[Developing a Robotic Claw and Walking Algorithm](https://www.malekhammoud.com/articles/walking-robot)** - Detailed walkthrough of the robotic claw control system and walking algorithm development
- **[The Anatomy-Drone: Understanding the Key Components](https://www.malekhammoud.com/articles/Anatomy-Drone)** - Overview of drone components and systems
- **[Building an Autonomous Drone with Raspberry Pi & Pixhawk](https://www.malekhammoud.com/articles/rpi-pixhawk-drone)** - Technical implementation guide for autonomous drone development

### Research Paper:
- **[Autonomous Litter Detection and Recovery System](https://www.malekhammoud.com/Autonomous_Litter_Detection_and_Recovery_System.pdf)** - Complete research paper documenting the project methodology and results

### Video Demo:
- **[Walking Robot Demo](https://youtu.be/L3JwvKmZ_t4)** - Live demonstration of the walking robot system in action

---
