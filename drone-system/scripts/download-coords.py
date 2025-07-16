# Connect to the vehicle
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from picamera2 import Picamera2
import cv2
import numpy as np
import time

# Connect to the vehicle
vehicle = connect('/dev/ttyS0',baud=57600,wait_ready=True)

def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

mission = download_mission()

for i in mission:
    print(i)
vehicle.close()

