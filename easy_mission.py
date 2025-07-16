# Connect to the vehicle
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import math

# Connect to the vehicle
vehicle = connect('/dev/ttyS0',baud=57600,wait_ready=True)

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

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# Go to next waypoint
def go_to_next_waypoint(cmd):
    target_location = LocationGlobalRelative(cmd['x'], cmd['y'], cmd['z'])
    vehicle.simple_goto(target_location)

# Main mission loop with real-time camera streaming
def run_mission():
    coord = {"x": 43.02994404434357, "y": -81.29301979819846, "z": 8.0}
    #coord = {"x": 43.02926066847666, "y": -81.29418570656019, "z": 8.0}
    print(coord["x"])
    go_to_next_waypoint(coord)

    while True:
        distance = get_distance_metres(vehicle.location.global_relative_frame, LocationGlobalRelative(coord['x'], coord['y'], coord['z']))
        print(f"Distance to waypoint: {distance:.2f} meters")

        if distance < 2.0:  # Arrived at waypoint
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
