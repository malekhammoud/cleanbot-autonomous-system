import time
from pymavlink import mavutil

while True:
    drone= mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

    # Wait for the heartbeat
    drone.wait_heartbeat()
    time.sleep(1)

    lat = drone.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
    lon = drone.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
    print(lat)
    print(lon)

