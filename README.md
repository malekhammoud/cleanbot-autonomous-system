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
│   │   ├── takeoff_and_land.py     # Basic flight ops
│   │   ├── easy_mission.py         # Simple missions
│   │   ├── guided_mission.py       # Advanced missions
│   │   ├── full.py                 # Complete automation
│   │   └── ... (6 more scripts)    # Testing and utility scripts
│   ├── data/                       # 📊 Organized mission data
│   │   ├── videos/                 # 🎥 Flight recordings (6 MP4 files)
│   │   ├── gps_logs/               # 🗺️ GPS trajectory data (5 CSV files)
│   │   ├── images/                 # 📸 Captured & processed images
│   │   │   └── captured/           # Raw mission images
│   │   └── databases/              # 🗃️ SQLite coordinate storage
│   ├── config/                     # ⚙️ MAVLink configuration
│   │   ├── mav.parm                # MAVLink parameters
│   │   ├── mav.tlog                # Telemetry logs
│   │   └── mav.tlog.raw            # Raw telemetry data
│   ├── logs/                       # 📋 System logs
│   └── README.md                   # Drone system documentation
├── walking-robot/                  # 🤖 Ground-based robot control
│   ├── ros2_ws_original/           # 🔧 Original robot package
│   │   └── src/robot/              # Basic motor control
│   ├── ros2_ws_robot/              # 🆕 Enhanced walking robot
│   │   └── src/robot_walking/      # Advanced walking control
│   └── README.md                   # Robot system documentation
├── .gitignore                      # Git ignore configuration
└── README.md                       # This file
```

## 🛠️ System Components

### 🚁 Drone System
- **Autonomous Flight Control**: GPS-based navigation and waypoint following
- **Computer Vision**: Real-time litter detection using OpenCV
- **Data Collection**: Synchronized GPS, video, and image capture
- **Mission Planning**: Automated survey patterns and area coverage
- **MAVLink Integration**: Direct drone communication and control

### 🤖 Walking Robot System
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
python live-detect.py       # Real-time detection
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
- GPS module
- Telemetry radio

### Robot Setup
- Raspberry Pi 4B
- Stepper motors for locomotion
- Servo motors for manipulation
- GPIO breakout board
- Power distribution system

## 🔧 Dependencies

### Core Libraries
- **ROS2 Humble** (Robot control)
- **OpenCV** (Computer vision)
- **MAVLink** (Drone communication)
- **RPi.GPIO** (Hardware control)
- **NumPy** (Data processing)
- **SQLite3** (Data storage)

### Installation
```bash
pip install opencv-python mavlink numpy sqlite3 rpi.gpio
sudo apt install ros-humble-desktop
```

## 📊 Features

- ✅ **Autonomous Navigation**: GPS-based flight planning
- ✅ **Real-time Detection**: Computer vision litter identification
- ✅ **Data Logging**: Comprehensive mission data storage
- ✅ **Multi-platform**: Drone and ground robot coordination
- ✅ **Modular Design**: Easy to extend and customize
- ✅ **Professional Organization**: Clean, documented codebase

## 📚 Documentation

- [`drone-system/README.md`](drone-system/README.md) - Detailed drone setup
- [`walking-robot/README.md`](walking-robot/README.md) - Robot control guide
- Individual script documentation in respective directories

## 🏆 Science Fair Impact

This project demonstrates:
- **Environmental Awareness**: Automated litter detection and mapping
- **Robotics Integration**: Multi-platform autonomous systems
- **Data Science**: GPS tracking and analysis
- **Computer Vision**: Real-time image processing
- **Engineering Excellence**: Professional code organization

---

*Built with ❤️ for environmental conservation and robotics education*
