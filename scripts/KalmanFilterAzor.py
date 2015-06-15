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

robot = Point(2,2,3);
top = None;
usbl = None;
dvl = None;
pressure = None;
imu = None;
groundTruth = None;

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
    
def getUSBLReading():
    dvlTime = time.time();
    global robot;
    global groundTruth;
    global imu;
    global dvl;
    global top;
    global usbl;
    
    pub = rospy.Publisher('/uwsim/KF_AZOR', Point, queue_size=10)
    rospy.init_node('KF_AZOR', anonymous=True)
    
    rospy.Subscriber("/uwsim/GPS_TOP", NavSatFix, setTop)
    rospy.Subscriber("/uwsim/usbl_AZOR", USBL, setUSBL)
    rospy.Subscriber("/uwsim/DVL_AZOR", DVL, setDVL)
    rospy.Subscriber("/uwsim/PRESSURESENSOR_AZOR", Pressure, setPressure)
    rospy.Subscriber("/uwsim/IMU_AZOR", Imu, setIMU)
    rospy.Subscriber("/uwsim/odom_AZOR", Odometry, setGroundTruth)
    
    rate = rospy.Rate(20) 
    while not rospy.is_shutdown():
        pointFromUSBL = None;
        groundTruthPosition = None;
       
        if(pressure):
            robot.z = pressure.pressure
            
        if(top and usbl and usbl.distance.distance and usbl.bearing.bearing):
            topPoint = [top.latitude, top.longitude, 2]
            pointFromUSBL = getPointFromUSBLReading(topPoint, usbl.bearing.bearing, usbl.distance.distance, robot.z)

        
        if(dvl and imu):
            newTime = time.time();
            xAxisSpeed = dvl.bi_x_axis * (newTime - dvlTime);
            yAxisSpeed = dvl.bi_y_axis * (newTime - dvlTime);
            zAxisSpeed = dvl.bi_z_axis * (newTime - dvlTime);
            dvlTime = newTime;
            
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
            
            deadReckoning =  numpy.mat(rotMatrix) * numpy.mat(speeds) ;
            
            robot.x = robot.x + deadReckoning[0][0];
            robot.y = robot.y + deadReckoning[1][0];
            

            if (groundTruth and pointFromUSBL):
                groundTruthPosition = groundTruth.pose.pose.position;
                print "truth x: ", round(groundTruthPosition.x, 3), " robot x: ", robot.x, " USBL ", pointFromUSBL.x;
                print "truth y: ", round(groundTruthPosition.y, 3), " robot y: ", robot.y, " USBL ", pointFromUSBL.y;
                print "truth z: ", round(groundTruthPosition.z, 3), " robot z: ", robot.z, " USBL ", pointFromUSBL.z;
                
                error = math.sqrt( (groundTruthPosition.x - robot.x)**2 + (groundTruthPosition.y - robot.y)**2+(groundTruthPosition.z - robot.z)**2 )
                usblerror =  math.sqrt( (groundTruthPosition.x - pointFromUSBL.x)**2 + (groundTruthPosition.y - pointFromUSBL.y)**2+(groundTruthPosition.z - pointFromUSBL.z)**2 )
                print "odomerror ", error, " usblerror ", usblerror,  "\n"
                pointFromUSBL = None;
            dvl = None;
            imu = None;


        pub.publish(robot)
        
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        getUSBLReading()
    except rospy.ROSInterruptException:
        pass