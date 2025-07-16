# Drone System for Litter Automation

This directory contains the complete drone control system for automated litter detection and collection.

## Directory Structure

### `scripts/`
Contains all Python scripts for drone operation:
- `main.py` - Main control script
- `drone.py` - Drone control class
- `control.py` - Flight control logic
- `gps-camera-record.py` - GPS-enabled camera recording
- `camera-segmentation.py` - Image processing for litter detection
- `live-detect.py` - Real-time litter detection
- `takeoff_and_land.py` - Basic flight operations
- `easy_mission.py` - Simple mission planning
- `guided_mission.py` - Advanced mission control
- `full.py` - Complete automation system

### `data/`
Organized data storage:

#### `videos/`
- Recorded flight videos
- Camera footage from missions
- Test recordings

#### `gps_logs/`
- GPS coordinate logs from flights
- Timestamp-matched location data
- Mission trajectory files

#### `images/`
- Captured images from drone camera
- Processed litter detection results
- Test images

##### `captured/`
- Raw images captured during missions

#### `databases/`
- SQLite databases for coordinate storage
- Mission data persistence
- Detection results storage

### `config/`
Configuration files:
- `mav.parm` - MAVLink parameters
- `mav.tlog` - Telemetry logs
- Flight configuration files

### `logs/`
System logs and debugging information

## Usage

### Basic Flight Operations
```bash
cd scripts
python takeoff_and_land.py
```

### Start Litter Detection Mission
```bash
cd scripts
python main.py
```

### Camera Stream Testing
```bash
cd scripts
python camera-stream-test.py
```

### GPS Recording
```bash
cd scripts
python gps-camera-record.py
```

## Hardware Requirements

- Drone with MAVLink support
- Camera module
- GPS module
- Companion computer (Raspberry Pi recommended)
- Telemetry radio

## Dependencies

- OpenCV
- MAVLink
- NumPy
- SQLite3
- Python 3.8+

## Features

- Autonomous flight control
- Real-time litter detection
- GPS-tagged image capture
- Mission planning and execution
- Data logging and analysis
