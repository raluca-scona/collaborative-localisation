#!/usr/bin/env python

from multi_robot_localisation.srv import *
from multi_robot_localisation.msg import *
from geometry_msgs.msg import Point
import numpy;
import math;
from common import *

def getPositionFromLBL(xypoints, xydistances):
    r = numpy.zeros((len(xypoints), 1));
        
    for i in range(len(r)):
        r[i] = 1./2*(xypoints[i] * xypoints[i].T - xydistances[i] ** 2)
        
    pseudoInverse = numpy.linalg.pinv(xypoints);
   
    onevector = numpy.ones((len(xypoints), 1))
    
    u = numpy.squeeze(numpy.asarray(pseudoInverse * onevector));
    v = numpy.squeeze(numpy.asarray(pseudoInverse * r));
    
    uudot = numpy.dot(u,u)
    uvdot = numpy.dot(u,v)
    vvdot = numpy.dot(v,v)
    
    lambdaSolutions = numpy.roots([uudot, 2 * (uvdot - 1), vvdot])
    
    xSolutions = numpy.zeros((len(lambdaSolutions), 2))
    for i in range(len(lambdaSolutions)):
        xSolutions[i] = u * lambdaSolutions[i] + v
    return xSolutions

    
def getPointFromLBLReadings(lbl1Dist, lbl2Dist, zRobot):
    print "distances: ",lbl1Dist, lbl2Dist

    if (lbl1Dist == None or lbl1Dist == 0 or lbl2Dist == None or lbl2Dist == 0):
        return None;
        
    lbl1 = getLBL1();
    lbl2 = getLBL2();
    
    lbls = numpy.concatenate((numpy.asmatrix(lbl1), numpy.asmatrix(lbl2)));
    xylbls = lbls[:, 0:2];
    xydistances = numpy.zeros((len(xylbls), 1));
    if (lbl1Dist ** 2 - (lbl1[2] - zRobot)**2 < 0):
        return None;
    if (lbl2Dist ** 2 - (lbl2[2] - zRobot)**2 < 0):
        return None;
    
    
    xydistances[0] = math.sqrt(lbl1Dist ** 2 - (lbl1[2] - zRobot)**2)
    xydistances[1] = math.sqrt(lbl2Dist ** 2 - (lbl2[2] - zRobot)**2)
    
    solutions = getPositionFromLBL(xylbls, xydistances)
    sol1 = None;
    sol2 = None;
    if (numpy.size(solutions, axis=0) == 2):
        sol1 = Point(solutions[0,0], solutions[0, 1], zRobot)
        sol2 = Point(solutions[1,0], solutions[1, 1], zRobot)
    elif (numpy.size(solutions, axis=0) == 1):
        sol1 = Point(solutions[0,0], solutions[0, 1], zRobot)
        sol2 = Point(solutions[0,0], solutions[0, 1], zRobot)

    return (sol1, sol2)