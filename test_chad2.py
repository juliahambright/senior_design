
from subprocess import PIPE
import subprocess
import sys
import re
import os
import math
#sys.path.insert(1,"~/librealsense/build/wrappers/python")
import pyrealsense2 as rs
from xbee import ZigBee
import serial

def get_lat2_lon2(currentLat, currentLon, dist, theta):
    R = 6378.1 #Radius of the Earth

    #lat2  52.20444 - the lat result I'm hoping for
    #lon2  0.36056 - the long result I'm hoping for.
    
    currentLat = float(currentLat)
    currentLon = float(currentLon)
    dist = float(dist)
    theta = float(theta)

    lat1 = math.radians(currentLat) #Current lat point converted to radians
    lon1 = math.radians(currentLon) #Current long point converted to radians

    lat2 = math.asin( math.sin(lat1)*math.cos(dist/R) + math.cos(lat1)*math.sin(dist/R)*math.cos(theta))

    lon2 = lon1 + math.atan2(math.sin(theta)*math.sin(dist/R)*math.cos(lat1),math.cos(dist/R)-math.sin(lat1)*math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    
    list = [lat2,lon2]
    return list

#----------------------------Main Program----------------------------

#setting up tracking camera
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
#finish camera setup



#setup XBee port
PORT = "/dev/ttyUSB0"
xbee_port = serial.Serial(PORT,9600)
xbee = ZigBee(xbee_port)



#determines if bot is inside or outside
inside = 1

if inside:
    pipe.start(cfg)

j = 0

#waypoint tree that saves the lists of waypoints detected in each locations by the lidar
wptree = {}

while(1):

    #get translation of camera frame data to pose data
    for _ in range(50):
        frames = pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        if pose:
            data = pose.get_pose_data()
            x = data.translation.x
            y = data.translation.y
            z = data.translation.z
            #print("data from get pose data: ")
            #print(data)
            #print("positionx: ")
            #print(data.translation.x)
    #to reset camera's position to 0,0,0
    #pipe.stop()
    
    lidar = subprocess.Popen([ "./ultra_simple"," /dev/ttyUSB1"], stderr=subprocess.STDOUT, stdout=subprocess.PIPE, shell=True)

    stdoutdata, stderrdata = lidar.communicate()
    print("stdoutdata: ",stdoutdata)
    print("stderrdata: ",stderrdata)
    print("x = ",x,"y = ",y,"z = ",z)

    match = re.findall("theta: (?P<theta>[\d.]+(?= )) , distance: (?P<distance>[\d.]+(?= )) , Waypoint (?P<waypoint>[\d.]+(?= ))", stdoutdata.decode('utf-8'))

    #if waypoint is found
    if len(match)>0:
    
        #add new location's waypoints to waypoint tree
        wptree["loc"+str(j)] = match
        theta = match[0][0]
        dist = match[0][1]
        print(theta)
        print(dist)

    #REMOVE THIS IF LATER!!!
    if True:
        currentLat = 52.20472
        currentLon = 0.14056

        #converts the distance and theta into a lattitude and longitude point relative to the current location
        #UNCOMMENT list
        #list = get_lat2_lon2(currentLat, currentLon, dist, theta)
        #nextLat = list[0]
        #nextLon = list[1]

        j=j+1
        #print(wptree)
        #REMOVE these nextLat and nextLon just testers
        nextLat = 30
        nextLon = 30
        waypoint2send = str(nextLat)+"," + str(nextLon)
        xbee.tx(dest_addr="\xFF\xFF",data=waypoint2send)

        xbee_port.close()
        


#result = subprocess.run(["./ultra_simple"," /dev/ttyUSB0"], stdout=PIPE, stderr=PIPE)
#print("result from c++: ",result.stdout.decode('utf-8'))
