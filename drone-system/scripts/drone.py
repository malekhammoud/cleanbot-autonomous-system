from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse

def connectdrone():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string= args.connect
    baud_rate = 115200
    vehicle = connect("/dev/ttyACM0", baud=baud_rate, wait_ready=True)
    return vehicle

def arm():
    time.sleep(5)
    """
    while vehicle.is_armable==False:
        print("waiting")
        time.sleep(1)
    print("armable")
    """
    vehicle.armed=True
    while vehicle.armed==False:
        print("waiting to arm")
        time.sleep(1)
    print("armed")
    return None

vehicle = connectdrone()
arm()
sleep(10)
time.print("end")

vehicle.simple_takeoff(10)

