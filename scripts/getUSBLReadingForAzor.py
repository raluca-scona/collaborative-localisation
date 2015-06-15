#!/usr/bin/env python

import sys
import rospy
import math
import numpy;
import random;
from multi_robot_localisation.srv import *
from multi_robot_localisation.msg import *
from simulateRange import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from common import *
from getPointFromUSBLReading import getPointFromUSBLReading

usbl = None
robot = None

def setRobot(data):
    position = data.pose.pose.position
    global robot 
    robot = numpy.array([position.x, position.y, position.z]);

#the USBL sensor is located at a depth of 2 metres      
def setUSBL(data):
    position = data.pose.pose.position
    global usbl 
    usbl = numpy.array([position.x, position.y, 2]);
    
def getUSBLReading():
    pub = rospy.Publisher('/uwsim/usbl_AZOR', USBL, queue_size=10)
    rospy.init_node('usbl_AZOR', anonymous=True)
    
    rospy.Subscriber("/uwsim/odom_AZOR", Odometry, setRobot)
    rospy.Subscriber("/uwsim/odom_TOP", Odometry, setUSBL)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if (usbl != None and robot != None):
            dist = getSimulatedDistanceReading(usbl, robot, 0.01)
            bearing = None;
            if (dist != None and dist != 0):
                bearingCos = getCosBetweenTwoVectors(usbl, robot)
                bearingSin = getSinBetweenTwoVectors(usbl, robot)
                bearing = numpy.arctan2(bearingSin, bearingCos)
                bearing = random.gauss(bearing, 0.0017)

            rospy.loginfo(USBL(Distance(dist), Bearing(bearing)))
            pub.publish(USBL(Distance(dist), Bearing(bearing)))
            
            #demo only
            print "Azor location: ", getPointFromUSBLReading(usbl, bearing, dist, robot[2])
            
            rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        getUSBLReading()
    except rospy.ROSInterruptException:
        pass