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
dvl = None;
imu = None;
odometry = Point(2,2,3);
        
def setDVL(data):
    global dvl;
    dvl = data;
    
def setIMU(data):
    global imu;
    imu = data;

def updateMotion(dvl, imu, timeDifference):
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
    
    return Point(deadReckoning.item(0,0), deadReckoning.item(1, 0), 0);

  
def runDeadReckoning():
  
    dvlTime = time.time();
    dr = Point(0,0,0);
    global imu;
    global dvl;
    
    pub = rospy.Publisher('/uwsim/DR_AZOR', Point, queue_size=10)
    rospy.init_node('DR_AZOR', anonymous=True)
    
    rospy.Subscriber("/uwsim/DVL_AZOR", DVL, setDVL)
    rospy.Subscriber("/uwsim/IMU_AZOR", Imu, setIMU)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
            
        if(dvl and imu):
            newTime = time.time();
            dr = updateMotion(dvl, imu, newTime - dvlTime);
            dvlTime = newTime;
                                                                                    
            dvl = None;
            imu = None;
        print dr.x, dr.y
        pub.publish(dr)
        
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        runDeadReckoning()
    except rospy.ROSInterruptException:
        pass