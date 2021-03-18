##########################################################
##  Some of the functions are taken from https://dronekit-python.
# readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
# #example-guided-mode-goto-position-target-local-ned
# which is the dronekit documentation
#
# Functions include: condition_yaw, goto_position_target_local_ned,

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import argparse
import os
import re
#import pyrealsense2 as rs

#------------------------------- Connect to Rover -----------------------------------#
#Connect to the vehicle
#connection_string = "tcp:127.0.0.1:5762"
connection_string = "/dev/ttyS0"
print("Connection to the vehicle on %s"%connection_string)
#TODO:change baud back to 57600 for rover
vehicle = connect(connection_string,baud = 57600, wait_ready=False)

#------------------------------- Get final location -----------------------------------#

final_dist = 15
final_angle = 240
"""
# Gets final distance and angle from command line
parser = argparse.ArgumentParser(description='Sets Vehicles final destination')
parser.add_argument('--dist',
                    help="Distance to Final Destination")
parser.add_argument('--angle',
                    help="Angle to Final Destination")

args = parser.parse_args()
#final_dist = args.dist
#final_angle = args.angle
"""

#------------------------------- Rover Functions ------------------------------------#

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
    #while not vehicle.is_armable:
        #time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
        vehicle.armed = True

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

    return data

#------------------------------- Angle Finding Functions ----------------------------#

# Finds the angle with no obstacles, closest to the final_angle 
def find_best_angle(gap_list, current_angle, lidar_read_dist, heading):
    min_edge_dist = 360

    j = 0
    for i in range (len(gap_list)):
        angle_range = gap_list[i]

        print("This is gap_list[i]: %s"%angle_range[0])
        print("This is gap_list[i]: %s"%angle_range[1])
        if float(angle_range[0]) == 0:
            if float(angle_range[1]) ==0:
                j = j+1
                print("GAP LIST HAS 0 in angle_range")
    if j == len(gap_list):
        print("GAP LIST FOUND ALL 0's\n")
        return current_angle

    #iterates through all gaps given by the lidar for a certian location 
    for i in range(len(gap_list)):
        # here we get the best angle for the individual given range 
        edge_dist, angle = get_edge_angle(gap_list[i], current_angle, lidar_read_dist, heading)

        #if the edge_dist is 0 that angle is best_angle
        if edge_dist == 0:
            best_angle = angle
            return best_angle

        #If the edge_dist is 1, the gap was less than 34 deg and greater than 11.3 deg so it picks the midpoint
        elif edge_dist ==1:

            best_angle = angle
            continue

        #if edge_dist is 2, the gap is less than 11.3 deg and should not use this gap
        elif edge_dist ==2:
            continue

        # if not we find the closest angle to the final angle from all ranges
        else:
            if edge_dist < min_edge_dist:
                min_edge_dist = edge_dist
                best_angle = angle

    best_angle =-1
    return best_angle


# If there is an obstacle, finds the edge of the gap closest to the final angle -- ASSUMES lidar reads 5 m

def get_edge_angle(angle_range, current_angle, lidar_read_dist, heading):
    # Calculates angle range buffer, this is for 1 meter away from the 
    # edge if the lidar range is 5 meters
    #print(lidar_read_dist)
    edge_buffer_meters = 1
    edge_buffer_rad = math.atan(edge_buffer_meters/lidar_read_dist)
    ##TODO: figure out how to calculate the edge_buffer based on lidar_read_dist
    #edge_buffer_degrees = (edge_buffer_rad*180)/(math.pi)
    #print("Edge Buffer rad: %s" %edge_buffer_rad)
    #print("Edge Buffer Degrees: %s" %edge_buffer_degrees)
    edge_buffer_degrees = 11.3

    #changes angle_range from lidar to cardinal degrees based on vehicles heading
    card_angle_range = lidar2cardinal_angle(angle_range, heading)

    gap_size = abs(card_angle_range[1]-card_angle_range[0])

    # If the gap size is too small for rover
    if gap_size < edge_buffer_degrees:
        return 2, 0

    ##TODO: change from hard coded to 3*edge_buffer_degrees
    # if the gap size is big enough for rover but too small for buffer, return midpoint
    if gap_size < 34:
        current_angle = (card_angle_range[1]+card_angle_range[0])/2
        return 1, current_angle

    #creates new angle range with buffer for obstacle
    card_angle_range[0] = card_angle_range[0] + edge_buffer_degrees
    card_angle_range[1] = card_angle_range[1] - edge_buffer_degrees

    #calculates distances from the edges of the range to the final angle
    edge_dist1 = abs(current_angle-card_angle_range[0])
    edge_dist2 = abs(current_angle-card_angle_range[1])

    # checks if the final angle is in one of the gaps
    if card_angle_range[0] > card_angle_range[1]:
        print("card angle range 0: %s" %card_angle_range[0])
        print("card angle range 1: %s" %card_angle_range[1])
        if current_angle>card_angle_range[0] or current_angle<card_angle_range[1]:
            print("(((In GAP)))-----weird case")
            return 0, current_angle

    if current_angle>card_angle_range[0] and current_angle<card_angle_range[1]:
        print("(((In GAP)))")
        return 0, current_angle

    # if not in range we return the closest edge
    if edge_dist1>edge_dist2:
        return edge_dist2, card_angle_range[1]
    else:
        return edge_dist1, card_angle_range[0]

def lidar2cardinal_angle(lidar_angle_range, heading):
    card_angle = [0,0]
    card_angle[0] = (int(float(lidar_angle_range[0])) + heading)%360
    card_angle[1] = (int(float(lidar_angle_range[1])) + heading)%360
    print("Cardinal Angle Gap: %s" %card_angle)
    return card_angle

#------------------------------- Math Functions ----------------------------#

def polar2cart(dist, angle):
    #angle in radians
    dist = float(dist)
    angle = float(angle)
    ang_rad = angle*(math.pi)/180
    x = dist*math.cos(ang_rad)
    y = dist*math.sin(ang_rad)

    return x,y

def cart2polar(current_x, current_y, final_x, final_y):
    #angle in radians
    x = float(final_x - current_x)
    y = float(final_y - current_y)
    dist = math.sqrt(x**2+y**2)
    angle_rad = math.atan(y/x)
    if (x < 0 and y < 0) or (x < 0 and y > 0):
        angle_rad = angle_rad + math.pi
    print(angle_rad)
    angle_degree = (angle_rad*180/(math.pi))%360

    return dist, angle_degree

def compare_dist(x1, y1, x2, y2):
    #x2,y2 should be the final coordinates
    #x1,y1 should be the waypoint to compare (or current location)
    y_diff = y2-y1
    x_diff = x2-x1
    dist = math.sqrt((y_diff**2) + (x_diff**2))
    return dist

def get_velo_duration(x_coord, y_coord, gndspd, lidarDist):
    x_velo = (x_coord*gndspd)/lidarDist
    y_velo = (y_coord*gndspd)/lidarDist
    duration = lidarDist/gndspd
    return x_velo, y_velo, duration

######################################################################################
#------------------------------- Main Code ------------------------------------------#
######################################################################################

# distance in meters, angle in degrees
#CHANGE
current_angle = final_angle%360
print(current_angle)
arm_and_takeoff()

#lidar dist in m
lidar_read_dist = 5

#groundspeed in m/s
gndspd = 1
vehicle.groundspeed = gndspd

# setting final dist and final x & y relative coords
dist_to_final = final_dist
final_x, final_y = polar2cart(final_dist, current_angle)

#starts with the current location as 0,0
current_x = 0
current_y = 0

##TODO: Im not using these vars right now -- see julia about em with lidar
wptree = {}
locations = {}

# Will stop if it has been a certain amount of time
start_time = time.time()
STOP_TIME = 60

#TO TRY!!!! vehicle.heading to know which direction it is pointed - this will be 0 degrees North
#so to know where the bot is, continuously update current_x and y with the amount it has traveled from the camera
#convert the camera's x and y (which will be relative to the bot's heading) into polar coordinates
#and then subtract the heading of the bot to get it on the North/East grid, 
#convert it back to x and y and then add that to current_x and current_y

##TODO: Fake Lidar data list -- remove when actually using Lidar and Pix
#--------------------------------------------

#--------------------------------------------


print("---------------------START-=-------------------------")
print("Final Relative x: %s" % final_x)
print("Final Relative y: %s\n" % final_y)

print("Current Relative x: %s" % current_x)
print("Current Relative y: %s\n" % current_y)


i = 0
##TODO: when used with Pix and Lidar get rid of FOR and replace with While TRUE or While with try and except
while True:
  try:
    print("Waiting for Lidar data...")
    lidar_data = get_lidar_data()

    match = re.findall("\n(?P<angle1>[\d.]+(?= )) (?P<angle2>[\d.]+(?= ))", lidar_data.decode('utf-8'))

    print("THIS IS MATCH: %s"%match)
    if match:

      # Finds polar to final from current position 
      current_dist, current_angle = cart2polar(current_x, current_y, final_x, final_y)
      print("\n ********************************************************* \n Current Best Heading to Final Loc: %s"% current_angle)            
      ##TODO: location is updated by i in the FOR loop which is getting taken out so must update this later
      print("------------------location : %s-------------------"%i)


      ##TODO: uncomment vehicle.heading -- right now hardcoding heading since not using Pix -- right now vehicle is in simulation change conn str to get on pix
      heading = vehicle.heading
      print("Current Vehicle Heading:||| %s ||| \n"% heading)
      #heading = 20

      # sets distance to point = lidar dist or dist to final if < lidar dist
      needed_dist = lidar_read_dist
      if dist_to_final < lidar_read_dist:
          print("^^^^^^^^In IF -- bot < 5m away^^^^^^^^^")
          needed_dist = int(math.floor(dist_to_final))
          print("dist to Final is: %s" %dist_to_final)
          print("velo dist to Final is: %s \n" %needed_dist)

      print("\n ------current angle: %s --------------"%current_angle)
      #finds best angle for single location of gap data -- for next goto command
      best_angle = find_best_angle(match, current_angle, needed_dist, heading)

      if best_angle==-1:
          print("-----------------BEST ANGLE IS -1 SO RETRYING LIDAR-----------\n")
          continue
      print("Best Angle: %s\n"%best_angle)

      # gets next x and y coord relative to current pos using best angle from above
      next_x, next_y = polar2cart(needed_dist, best_angle)
      print("Next x: %s" % next_x)
      print("Next y: %s \n" % next_y)

      # Gets Velocitys (in X (+N,-S) and Y(+E,-W)) and Duration, using x and y coords and gndspeed
      x_velo, y_velo, duration = get_velo_duration(next_x, next_y, gndspd, needed_dist)
      print("Next x velo: %s" % x_velo)
      print("Next y velo: %s" % y_velo)
      print("Next duration: %s \n" % duration)

      # sends MAVLink Message to Autopilot for the GUIDED go To command with the VELO and DURATION
      print("<Going to WP>")
      send_ned_velocity(x_velo, y_velo, 0 , duration)
      print("<Arrived at next WP>\n")

      # Keeping track of total relative location (in x y plane) to where we started 
      current_x = current_x + next_x
      current_y = current_y + next_y
      print("Current Relative x: %s" % current_x)
      print("Current Relative y: %s\n" % current_y)

      # finds distance to point from current location 
      dist_to_final = compare_dist(current_x, current_y, final_x, final_y)
      print("***Distance to Final Loc: %s***\n" %dist_to_final)

      locations["loc"+str(i)] = [current_x, current_y]
      i = i+1

  except KeyboardInterrupt or dist_to_final<1:
      break

    # 0.91 meters is 1 yard
    """
    if dist_to_final > 0.91:
        #Commented out lidar data
        #wplist = get_lidar_data()
        wplist = lidar_list[i]

        #if waypoint is found
        if len(wplist)>0:
            #orders the waypoints from closest to the final destination to farthest
            orderedWP = order_waypoints(wplist, final_x, final_y)
            
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
            #goto_position_target_local_ned(wp_x, wp_y, 0)

            ##TODO: take out time.sleep once velocity is implemented
            #time wait (speed, distance) - gives the bot time to arrive at waypoint
            time_hold = time_wait(gndspd, bestWP[1])
            time.sleep(time_hold)

            current_x = current_x + wp_x
            current_y = currenty + wp_y





if not elapsed > STOP_TIME:
    print("\n <<<<<<<<<<<<<BOT ARRIVED>>>>>>>>>>>>>>>>> \n")
"""

##TODO: get rid of RTL -- just for simulation
"""
print("RTL coming back")
vehicle.mode = "RTL"
time.sleep(15)
"""

# change to HOLD mode and disarm
change2_hold_mode()
disarm_vehicle()

# Print and Close the Vehicle
print("Vehicle Armed: %s" % vehicle.armed)
vehicle.close()
print("Vehicle Closed -- Mission Over")
time.sleep(5)
