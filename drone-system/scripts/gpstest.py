import time
from pymavlink import mavutil

# Connect to the Pixhawk via MAVLink
drone = mavutil.mavlink_connection('/dev/ttyS0', baud=57600)

# Wait for the heartbeat
drone.wait_heartbeat()
time.sleep(1)

while True:
    # Receive the next message
    msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    if msg:
        # Extract GPS data from the received message
        lat = msg.lat * 1.0e-7
        lon = msg.lon * 1.0e-7

        # Print GPS data
        print(f"Latitude: {lat}, Longitude: {lon}")

    # Optional: Add a small sleep to avoid flooding with too many requests
    time.sleep(0.001)

