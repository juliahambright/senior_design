##########################################################
##  Some of the functions are taken from https://dronekit-python.
# readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
# #example-guided-mode-goto-position-target-local-ned
# which is the dronekit documentation
#
# Functions include: condition_yaw, goto_position_target_local_ned,
# condition_yaw

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 
import time
import math
import argparse
import pyrealsense2 as rs

#------------------------------- Connect to Rover -----------------------------------#
#Connect to the vehicle
connection_string = "/dev/ttyS0"
print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string,baud = 57600, wait_ready=False)

######################################################################################
#------------------------------- Rover Functions ------------------------------------#
######################################################################################

def change2_guided_mode():
    while not vehicle.mode == "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
    print(vehicle.mode)
    

def change2_hold_mode():
    while not vehicle.mode == "HOLD":
        vehicle.mode = VehicleMode("HOLD")
        time.sleep(1)
    print(vehicle.mode)


def arm_vehicle():
    while not vehicle.is_armable:
        time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed: time.sleep(1)

def disarm_vehicle():
    vehicle.armed = False
    while vehicle.armed: time.sleep(1)

def arm_and_takeoff():
    print("Arming motors--2 sec wait")

    arm_vehicle()
    print("Vehicle Armed: %s\n"%vehicle.armed)

    # once armable vehile set to GUIDED mode
    change2_guided_mode()
    print("--Bot Mode-- %s" %vehicle.mode)


#------------------------------- Mavlink/Rover Functions ----------------------------#


def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def goto_position_target_local_relative(north, east, down):
    """	
    This function is relative to the bot's position

    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_relative_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    This function is relative to the bot's position

    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


#------------------------------- Lidar Function ----------------------------#

def get_lidar_data():
    data = os.popen('sudo ./ultra_simple').read()
    print("This is the Lidar output:\n %s"%data)
    #I added a time sleep so that the usb port has time to close
    #but it is probably not necessary 
    #REMOVE LATER
    time.sleep(2)
    match = re.findall("theta: (?P<theta>[\d.]+(?= )) , distance: (?P<distance>[\d.]+(?= )) , Waypoint (?P<waypoint>[\d.]+(?= ))", var.decode('utf-8'))
    
    return match

#------------------------------- Math Functions ----------------------------#

def polar2cart(dist, angle):
    #angle in radians
    dist = float(dist)
    angle = float(angle)
    ang_rad = angle*(math.pi)/180
    x = dist*math.cos(ang_rad)
    y = dist*math.sin(ang_rad)

    return x,y

def compare_dist(x1, y1, x2, y2):
    #x2,y2 should be the final coordinates
    #x1,y1 should be the waypoint to compare (or current location)
    y_diff = y2-y1
    x_diff = x2-x1
    dist = math.sqrt((y_diff**2) + (x_diff**2)) 


#----------------------------------------------------------------------------#

def order_waypoints(wps, final_x, final_y):
    #wps is in the form of a list [{angle, dist, waypoint}, {angle, dist, waypoint}]
    i = 0
    dist_list = []
    hold_dist_list = []
    orderedWP = []

    for i in range(len(wps)):
        #convert waypoint to cartesian coordinates
        x1,y1 = polar2cart(wps[i][1], wps[i][0])
        #the distance between waypoint i and the final location
        dist2final = compare_dist(x1, y1, final_x, final_y)
        #adds the distance to the distance list
        dist_list.append(dist2final)
    i = 0

    #holds the distances and deletes the minimum everytime (temporary)
    hold_dist_list = dist_list
    
    #creates a list of ordered waypoints based on distance
    for i in range(len(dist_list)):
        print("This is hold_dist_list: %s"%hold_dist_list)
        #every next smallest distance
        minm = min(hold_dist_list)
        #this is the index of the minimum in the original list of distances
        index = dist_list.index(minm)
        #this is the index of the minimum in the reduced list of distances
        index2 = hold_dist_list.index(minm)
        #Gets rid of the next smallest distance
        hold_dist_list.pop(index2)
        #appending the next best waypoint to order the waypoints
        orderedWP.append(wps[index])
    
    #the minimum of the distance list is the closest to the final destination
    print("Ordered Waypoint List: %s"%orderedWP)    
    #remove the bestWP and return the new WP list to be save for future use

    return orderedWP

def time_wait(gndspd, distance):
    time_wait = distance/gndspd + 2
    return time_wait

######################################################################################
#------------------------------- Main Code ------------------------------------------#
######################################################################################

# Gets final distance and angle from command line
parser = argparse.ArgumentParser(description='Sets Vehicles final destination')
parser.add_argument('--dist',
                    help="Distance to Final Destination")
parser.add_argument('--angle',
                    help="Angle to Final Destination")

args = parser.parse_args()
#final_dist = args.dist
#final_angle = args.angle

# distance in meters, angle in degrees
#CHANGE
final_dist = 10
final_angle = 30

arm_and_takeoff()

gndspd = 1
vehicle.groundspeed = gndspd

#Sets the initial relative position to 0,0,0
goto_position_target_local_ned(0,0,0)
condition_yaw(0)

dist_to_final = final_dist
final_x, final_y = polar2cart(final_dist, final_angle)

#starts with the current location as 0,0
current_x = 0
current_y = 0
wptree = {}
locations = {}
j = 0

# Will stop if it has been a certain amount of time
EMERGENCY_STOP = time.time()
STOP_TIME = 60

#TO TRY!!!! vehicle.heading to know which direction it is pointed - this will be 0 degrees North
#so to know where the bot is, continuously update current_x and y with the amount it has traveled from the camera
#convert the camera's x and y (which will be relative to the bot's heading) into polar coordinates
#and then subtract the heading of the bot to get it on the North/East grid, 
#convert it back to x and y and then add that to current_x and current_y


while True:
    elapsed = time.time() - EMERGENCY_STOP
    try:
        # 0.91 meters is 1 yard
        if dist_to_final > 0.91:
            wplist = get_lidar_data()

            #if waypoint is found
            if len(wplist)>0:
                #orders the waypoints from closest to the final destination to farthest
                orderedWP = order_waypoints(wplist, final_x, final_y):
                
                #waypoint tree saves all of the waypoints ever gathered at each location
                wptree["loc"+str(j)] = orderedWP
                # bestWP in the form [angle, distance, waypoint]
                bestWP = orderedWP[0]
                #locations saves the current coordinates that the waypoints were gathered at
                locations["loc"+str(j)] = [current_x, current_y]

                #best waypoint in cartesian
                wp_x, wp_y =  polar2cart(bestWP[1], bestWP[0])
                print("The next waypoint is x = %s\n "%wp_x)
                print("The next waypoint is y = %s\n "%wp_y)
                j = j+1

                ##TODO: change goto_position_target_local_ned
                print("----------------Vehicle is going to next waypoint-----------------\n")
                goto_position_target_local_ned(wp_x, wp_y, 0)

                #time wait (speed, distance) - gives the bot time to arrive at waypoint
                time_hold = time_wait(gndspd, bestWP[1])
                time.sleep(time_hold)

                current_x = current_x + wp_x
                current_y = currenty + wp_y

    except KeyboardInterrupt or elapsed>STOP_TIME:
        break



if not elapsed >STOP_TIME:
    print("\n <<<<<<<<<<<<<BOT ARRIVED>>>>>>>>>>>>>>>>> \n")

change2_hold_mode()
disarm_vehicle()

print("Vehicle Armed: %s" % vehicle.armed)
vehicle.close()
print("Vehicle Closed -- Mission Over")
time.sleep(5)
