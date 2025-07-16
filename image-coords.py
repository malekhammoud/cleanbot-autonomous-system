from picamera2 import Picamera2, Preview
from pymavlink import mavutil
import time
import sqlite3


picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
#picam2.start_preview(Preview.DRM)
picam2.start()
time.sleep(1)
print("START")


data= sqlite3.connect("coords.db")
print(data.total_changes)

cursor = data.cursor()



# Connect to the Pixhawk via MAVLink
drone = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for the heartbeat
drone.wait_heartbeat()
time.sleep(1)

try:
    while True:
        # Receive the next message
        msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

        if msg:
            # Extract GPS data from the received message
            lat = msg.lat * 1.0e-7
            lon = msg.lon * 1.0e-7

            # Print GPS data
            print(f"Latitude: {lat}, Longitude: {lon}")

            cursor.execute("SELECT * FROM coords ORDER BY id DESC LIMIT 1")
            # Fetch the result
            num = int(cursor.fetchone()[2])

            if not num:
                num=1
            else:
                num+= 1
            path = "./images/image"
            path += str(num) + ".jpg"

            picam2.capture_file(path)

            cursor.execute(f"INSERT INTO coords (lat, lon) VALUES ({lat}, {lon})")
            data.commit()
except:
    print('end')
    data.close()
