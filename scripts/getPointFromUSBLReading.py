#!/usr/bin/env python

from multi_robot_localisation.srv import *
from multi_robot_localisation.msg import *
from geometry_msgs.msg import Point
from common import *
    
def getPointFromUSBLReading(usbl, usblBearing, usblDistance, zRobot):
    if (usblBearing == None or usblDistance == None or usblDistance == 0):
        return None;
        
    xdisplacement = computeXDisplacement(usblBearing, usblDistance, usbl[2] - zRobot)
    ydisplacement = computeYDisplacement(usblBearing, usblDistance, usbl[2] - zRobot)
    robotLocation = Point(usbl[0] + xdisplacement, usbl[1] + ydisplacement, zRobot);    
    return robotLocation