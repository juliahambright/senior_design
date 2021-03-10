from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse
import math
import pymavlink
from pymavlink import mavutil

connection_string = "/dev/ttyS0"
#connection_string = "tcp:127.0.0.1:5762"
print("Connection to the vehicle on " + connection_string)
vehicle = connect(connection_string, baud=921600, wait_ready=True)

def get_distance_meters(aLoc1, aLoc2):
    dlat = aLoc2.lat - aLoc1.lat
    dlong = aLoc2.lon - aLoc1.lon
    return math.sqrt((dlat*dlat)+(dlong*dlong))*1.113195e5

def arm_and_takeoff(tgt_altitude):
    print("Arming motors -- 10 sec wait")

    time.sleep(2)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    print("Bot Armed and in Guided Mode")

    while not vehicle.armed: time.sleep(1)

    time.sleep(3)
    print("Vehicle Mode: %s" % vehicle.mode)
    print("Vehicle Armed: %s" % vehicle.armed)

#---------Main Program---------

arm_and_takeoff(10)


msg = vehicle.message_factory.vision_position_estimate_encode(1200,0,0,0,0,0,0)

print("this is msg: %s"%msg)
vehicle.send_mavlink(msg)

vehicle.groundspeed = .5
print("Groundspeed: "+ str(vehicle.groundspeed))

print("Setting Launch Location to Current Coordinates -- sleep 3 sec")
vehicle.home_location = vehicle.location.global_frame
print("Home Current Location: " + str(vehicle.location.global_frame))
time.sleep(3)

wp1 = LocationGlobalRelative(30.6232513, -96.3449646, 10)
print("wp1 set to: " + str(wp1))

print(">>>>>Going to wp1 -- wait 5 sec for bot to arrive<<<<<<<<")
vehicle.simple_goto(wp1)

time.sleep(15)

print("Current Location: " + str(vehicle.location.global_frame))
distance2wp = get_distance_meters(vehicle.location.global_relative_frame, wp1)
print("Distance to wp: " + str(distance2wp))

if distance2wp < 10:
    print("*************Bot Arrived***********")
    print("Global Location: " + str(vehicle.location.global_frame))

    print("Coming back -- RTL mode")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(10)
    print("Vehicle mode: " + str(vehicle.mode))

while not vehicle.mode == "HOLD":
    print("Vehicle Mode: " + str(vehicle.mode))
    vehicle.mode = VehicleMode("HOLD")
    time.sleep(1)

print("Vehicle Mode: " + str(vehicle.mode))
vehicle.close()
print("Vehicle Closed Mission Over")
time.sleep(5)

