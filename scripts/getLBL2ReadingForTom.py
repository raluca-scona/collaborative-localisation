#!/usr/bin/env python

from multi_robot_localisation.srv import *
from multi_robot_localisation.msg import *
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from simulateRange import *
import numpy;
import random;
import math;
import matplotlib.pyplot as plt
from common import *

robot = None;

def euclideanDistance(a, b):
    return numpy.linalg.norm(a - b);

def setRobot(data):
    position = data.pose.pose.position
    global robot 
    robot = numpy.array([position.x, position.y, position.z]);
    
def getLBLReading():
    pub = rospy.Publisher('/uwsim/LBL2_TOM', LBL, queue_size=10)
    rospy.init_node('LBL2_TOM', anonymous=True)
    
    rospy.Subscriber("/uwsim/odom_TOM", Odometry, setRobot)
    
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        if (robot != None):

            dist = getSimulatedDistanceReading(getLBL2(), robot, 0.1)
            rospy.loginfo(LBL(Distance(dist)))
            pub.publish(LBL(Distance(dist)))
            
            rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        getLBLReading()
    except rospy.ROSInterruptException:
        pass