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
groundTruth = None;
pf = None;
ic = None;
nls = None;
        
def setGroundTruth(data):
    global groundTruth;
    groundTruth = data;
    
def setPF(data):
    global pf;
    pf = data;

def setIC(data):
    global ic;
    ic = data;
    
def setNLS(data):
    global nls;
    nls = data;
    
def runAllLocalizations():
    pfdata = open('pf.txt', 'w');
    nlsdata = open('nls.txt', 'w');
    icdata = open('ic.txt', 'w');

    truthdata = open('truth.txt', 'w');
    
    startTime = time.time();
    global groundTruth;
    global pf;
    global ic;
    global nls;
    
    #pub = rospy.Publisher('/uwsim/ALL_AZOR', Point, queue_size=10)
    rospy.init_node('ALL_AZOR', anonymous=True)
    

    rospy.Subscriber("/uwsim/IC_AZOR", Point, setIC)
    rospy.Subscriber("/uwsim/PFRANGE_AZOR", Point, setPF)
    rospy.Subscriber("/uwsim/NLSRANGE_AZOR", Point, setNLS)
    rospy.Subscriber("/uwsim/odom_AZOR", Odometry, setGroundTruth)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print time.time() - startTime
        if(pf):
            pfdata.write(`pf.x` + " " + `pf.y` + " " + `pf.z` + "\n")
            pf = None;
        if(ic):
            icdata.write(`ic.x` + " " + `ic.y` + " " + `ic.z` + "\n")
            ic = None;
        if(nls):
            nlsdata.write(`nls.x` + " " + `nls.y` + " " + `nls.z` + "\n")
            nls = None;
        if(groundTruth):
            pos = groundTruth.pose.pose.position;
            truthdata.write(`pos.x` + " " + `pos.y` + " " + `pos.z`+ "\n");
            groundTruth = None;
        
        if (time.time() - startTime > 900):
            pfdata.close();
            truthdata.close();
            icdata.close();
            nlsdata.close();
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        runAllLocalizations()
    except rospy.ROSInterruptException:
        pass