from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from picamera2 import Picamera2
import cv2
import numpy as np
import time
import math

# Connect to the vehicle
vehicle = connect('/dev/ttyS0', baud=57600, wait_ready=True)

# Initialize camera
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(camera_config)
picam2.start()

# Arm and takeoff function
def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def download_mission():
    """
    Downloads the current mission and returns it in a list.
    """
    print("Download mission from vehicle")
    missionlist = []
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

# Go to next waypoint
def go_to_next_waypoint(cmd):
    target_location = LocationGlobalRelative(cmd.x, cmd.y, cmd.z)
    vehicle.simple_goto(target_location)

# Detect litter in a video frame
def detect_litter(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.medianBlur(hsv, 11)

    # Define the color range for masking (adjust for litter detection)
    lower = np.array([27, 3, 107])
    upper = np.array([98, 102, 255])

    # Create a mask and apply it to the image
    mask = cv2.inRange(blur, lower, upper)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contour is detected as litter
    for contour in contours:
        if cv2.contourArea(contour) > 10:  # Adjust area threshold
            return 1
    return 0
# Main mission loop with real-time camera streaming
def run_mission():
    mission = download_mission()

    for i, cmd in enumerate(mission):
        if cmd.frame == 3: 
            print(f"Executing waypoint {i + 1}/{len(mission)}")
            go_to_next_waypoint(cmd)

            while True:
                distance = get_distance_metres(vehicle.location.global_relative_frame, LocationGlobalRelative(cmd.x, cmd.y, cmd.z))
                print(f"Distance to waypoint {i + 1}: {distance:.2f} meters")

                # Get camera frame
                frame = picam2.capture_array()

                # If litter is detected, stop the drone for 4 seconds
                if detect_litter(frame):
                    print("Litter detected!")
                    vehicle.mode = VehicleMode("LOITER")  # Change to LOITER mode
                    time.sleep(6)
                    vehicle.mode = VehicleMode("GUIDED")  # Change back to GUIDED mode
                    continue  # Continue to next iteration

                # Break if the distance to the waypoint is less than 2.0 meters
                if distance < 2.0:
                    print(f"Arrived at waypoint {i + 1}")
                    break

                time.sleep(0.1)

# Takeoff to a target altitude
arm_and_takeoff(8)

# Run the mission with real-time camera feed
run_mission()

# Land after mission
print("Mission complete, landing...")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
vehicle.close()
picam2.stop()  # Stop the camera before exiting
