#!/usr/bin/env python

from multi_robot_localisation.srv import *
from multi_robot_localisation.msg import *
import rospy
from nav_msgs.msg import Odometry
from simulateRange import *
import numpy;
from common import *
from getPointFromLBLReadings import getPointFromLBLReadings

robot = None;
lbl2Reading = None;

def setRobot(data):
    position = data.pose.pose.position
    global robot 
    robot = numpy.array([position.x, position.y, position.z]);

##for demo purposes
def setLBL2Depth(data):
    global lbl2Reading 
    lbl2Reading = data.lbl.distance
    
def getLBLReading():
    pub = rospy.Publisher('/uwsim/LBL1_AZOR', LBL, queue_size=10)
    rospy.init_node('LBL1_AZOR', anonymous=True)
    
    rospy.Subscriber("/uwsim/odom_AZOR", Odometry, setRobot)
    
    #for demo purposes
    rospy.Subscriber("/uwsim/LBL2_AZOR", LBL, setLBL2Depth)
    
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        if (robot != None):
            dist = getSimulatedDistanceReading(getLBL1(), robot, 0.1)
            #rospy.loginfo(LBL(Distance(dist)))
            pub.publish(LBL(Distance(dist)))
            
            ##for demo purposes
            print getPointFromLBLReadings(dist, lbl2Reading, robot[2])
            
            rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        getLBLReading()
    except rospy.ROSInterruptException:
        pass