#!/usr/bin/env python

import sys
import rospy
import math
import numpy;
import random;
import time;
from multi_robot_localisation.srv import *
from multi_robot_localisation.msg import *
from simulateRange import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import NavSatFix, Imu
from underwater_sensor_msgs.msg import DVL, Pressure
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from common import *
from getPointFromUSBLReading import getPointFromUSBLReading

usblDepth = 2;
location = numpy.array([ [0., 0.], [0., 0.], [0., 0.] ]);
top = None;
usbl = None;
dvl = None;
pressure = None;
imu = None;
groundTruth = None;
usblPoints = {};
usblStandardDev = 0.2;
usblx = numpy.array([]);
usbly = numpy.array([]);
odometry = Point(2,2,3);

def getIntersectionOfIntervals(a,b,c,d):
   # print a,b,c,d

    if ((c < a and d < a) or (a < c and b < c)):
        print "no intersection"
        return [min(a,c), max(b, d)]

    elif (c<a):
        if (d<b):
            return [a,d];
        else:
            return [a,b];
    elif (b<d):
        return [c,b];
    return [c,d];

def setTop(data):
    global top;
    top = data;
    
def setUSBL(data):
    global usbl;
    usbl = data;
        
def setDVL(data):
    global dvl;
    dvl = data;
    
def setPressure(data):
    global pressure;
    pressure = data;
    
def setIMU(data):
    global imu;
    imu = data;

def setGroundTruth(data):
    global groundTruth;
    groundTruth = data;
    
def initLocation(initPoint):
    location = numpy.array([ [initPoint.x - 10., initPoint.x + 50.], [initPoint.y - 10., initPoint.y + 5.], [initPoint.z - 0.1, initPoint.z + 0.1]  ]);
    return location;

def updateMotion(location, dvl, imu, timeDifference):
    newLocation = numpy.array([ [0.0, 0.0], [0.0, 0.0], [ location[2][0], location[2][1] ] ]);
    global odometry;
    
    xAxisSpeed = dvl.bi_x_axis * timeDifference;
    yAxisSpeed = dvl.bi_y_axis * timeDifference;
    zAxisSpeed = dvl.bi_z_axis * timeDifference;    
    
    speeds = numpy.array([ [xAxisSpeed], [yAxisSpeed], [zAxisSpeed] ]);
    qx = imu.orientation.x;
    qy = imu.orientation.y;
    qz = imu.orientation.z;
    qw = imu.orientation.w;
     
    rotMatrix = numpy.array([ [0.,0., 0.], [0.,0., 0.], [0.,0., 0.] ]);
    rotMatrix[0][0] = 1 - 2*qy**2 - 2*qz**2;
    rotMatrix[0][1] = 2*qx*qy - 2*qz*qw;
    rotMatrix[0][2] = 2*qx*qz + 2*qy*qw;
    
    rotMatrix[1][0] = 2*qx*qy + 2*qz*qw;
    rotMatrix[1][1] = 1 - 2*qx**2 - 2 *qz**2;
    rotMatrix[1][2] = 2*qy*qz - 2*qx*qw;
    
    rotMatrix[2][0] = 2*qx*qz - 2*qy*qw;
    rotMatrix[2][1] = 2*qy*qz + 2*qx*qw;
    rotMatrix[2][2] = 1 - 2*qx**2 - 2*qy**2;
    
    deadReckoning =  numpy.mat(rotMatrix) * numpy.mat(speeds);
    
    odometry.x += deadReckoning.item(0,0);
    odometry.y += deadReckoning.item(1,0);
    
    newLocation[0][0] = location[0][0] + deadReckoning.item(0,0) - 0.05;
    newLocation[0][1] = location[0][1] + deadReckoning.item(0,0) + 0.05;
    
    newLocation[1][0] = location[1][0] + deadReckoning.item(1,0) - 0.05;
    newLocation[1][1] = location[1][1] + deadReckoning.item(1,0) + 0.05;
          
    return newLocation;
    
def getMedianPoint(usblPoints):
    distances = usblPoints.keys();
    distances.sort();
    
    return usblPoints[distances[1]];
    
location = initLocation(Point(2,2,3));

  
def runIntervalCalculus():
    icdata = open('ic.txt', 'w');
    usbldata = open('usbl.txt', 'w');
    odometrydata = open('odometry.txt', 'w');
    truthdata = open('truth.txt', 'w');  
    icerrors = open('icerr.txt', 'w');

    
    dvlTime = time.time();
    startTime = time.time();
    global location;
    global groundTruth;
    global imu;
    global dvl;
    global top;
    global usbl;
    global usblPoints;
    global usblx;
    global usbly;
    
    pub = rospy.Publisher('/uwsim/IC_AZOR', Point, queue_size=10)
    rospy.init_node('IC_AZOR', anonymous=True)
    
    rospy.Subscriber("/uwsim/GPS_TOP", NavSatFix, setTop)
    rospy.Subscriber("/uwsim/usbl_AZOR", USBL, setUSBL)
    rospy.Subscriber("/uwsim/DVL_AZOR", DVL, setDVL)
    rospy.Subscriber("/uwsim/PRESSURESENSOR_AZOR", Pressure, setPressure)
    rospy.Subscriber("/uwsim/IMU_AZOR", Imu, setIMU)
    rospy.Subscriber("/uwsim/odom_AZOR", Odometry, setGroundTruth)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pointFromUSBL = None;
        
        if(pressure):
            location[2, 0] = pressure.pressure;
            location[2, 1] = pressure.pressure;
                               
        if(top and usbl and usbl.distance.distance and usbl.bearing.bearing):
            topPoint = [top.latitude, top.longitude, 2];
            pointFromUSBL = getPointFromUSBLReading(topPoint, usbl.bearing.bearing, usbl.distance.distance, numpy.mean(location[2]))
            usblx = numpy.append(usblx, pointFromUSBL.x);
            usbly = numpy.append(usbly, pointFromUSBL.y)
            top = None;
            usbl = None;
            #pointFromUSBL = None;
            
        if(dvl and imu):
            newTime = time.time();
            location = updateMotion(location, dvl, imu, newTime - dvlTime);
            dvlTime = newTime;
                       
            if (pointFromUSBL):
                print 'got usbl point'
                filteredUsblx = None;
                filteredUsbly = None;
                #if (len(usblx) > 50):
                filteredUsblx = savitzky_golay(usblx, 9, 3);
                filteredUsbly = savitzky_golay(usbly, 9, 3);
                    #print pointFromUSBL.x, filteredUsblx;
                    #pointFromUSBL.x = filteredUsbly[len(filteredUsblx)-1];
                    #pointFromUSBL.y = filteredUsbly[len(filteredUsbly)-1];                
                
                #print "usbl reading ", pointFromUSBL.x, pointFromUSBL.y, pointFromUSBL.z;
                xInterval = getIntersectionOfIntervals(location[0,0], location[0,1], pointFromUSBL.x - 0.15, pointFromUSBL.x + 0.15);
                yInterval = getIntersectionOfIntervals(location[1,0], location[1,1], pointFromUSBL.y - 0.15, pointFromUSBL.y + 0.15);
                
                location[0,0] = xInterval[0];
                location[0,1] = xInterval[1];
                location[1,0] = yInterval[0];
                location[1,1] = yInterval[1];
                '''print time.time() - startTime
                if (time.time() - startTime < 150):
                    averageLocation = Point(numpy.mean(location[0]), numpy.mean(location[1]), numpy.mean(location[2]))
                    icdata.write(`pointFromUSBL.x` + " " + `pointFromUSBL.y` + " " + `pointFromUSBL.z` + "\n")
                    odometrydata.write(`odometry.x` + " " + `odometry.y` + " " + `odometry.z`+ "\n")
                    pos = groundTruth.pose.pose.position;
                    truthdata.write(`pos.x` + " " + `pos.y` + " " + `pos.z`+ "\n");
                    usbldata.write(`pointFromUSBL.x` + " " + `pointFromUSBL.y` + " " +`pointFromUSBL.z`+ "\n");
                else:
                    icdata.close();
                    odometrydata.close();
                    truthdata.close();
                    usbldata.close();'''
                                                 
                pointFromUSBL = None;
            
            dvl = None;
            imu = None;
       
        averageLocation = Point(numpy.mean(location[0]), numpy.mean(location[1]), numpy.mean(location[2]))
        
        if (groundTruth):
            pos = groundTruth.pose.pose.position
            error = math.sqrt((averageLocation.x - pos.x)**2 +  (averageLocation.y - pos.y)**2+(averageLocation.z - pos.z)**2 );
            print pos.x, pos.y, pos.z, error;
            icerrors.write(`error` + "\n");


        pub.publish(averageLocation)
        
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        runIntervalCalculus()
    except rospy.ROSInterruptException:
        pass