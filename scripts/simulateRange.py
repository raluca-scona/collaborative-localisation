import random;
import math;
import matplotlib.pyplot as plt
import numpy;
from common import *

    
def probSurfaceMultiPath(robot_z):
    #sigmoid centred at 2 meaning in a depth of 2 metres, the probability of multipath is 0.5
    return 1. / (1. + math.exp(robot_z-2))

def probBottomMultiPath(robot_z):
    distanceToSeaFloor = getSeaDepth() - robot_z;    
    return 1. / (1. + math.exp(distanceToSeaFloor-2))
    
def selectMultipathContact(sensor, robot, depth):
    contact = [0,0, depth]
    if (sensor[0] == robot[0] and sensor[1] == robot[1]):
        contact[0] = sensor[0];
        contact[1] = sensor[1];
    elif (sensor[0] == robot[0] and sensor[1] != robot[1]):
        contact[0] = sensor[0];
        contact[1] = random.uniform(sensor[1], robot[1]);
    elif(sensor[0] != robot[0] and sensor[1] == robot[1]):
        contact[0] = random.uniform(sensor[0], robot[0]);
        contact[1] = sensor[1];
    else:
        contact[0] = random.uniform(sensor[0], robot[0]);
        contact[1] = (robot[1] - sensor[1]) * 1. / (robot[0] - sensor[0]) * (contact[0] - sensor[0]) + sensor[1];
    return contact;
        
def simulateMultiPath(sensor, robot, depth):
    contact = selectMultipathContact(sensor, robot, depth);
    return euclideanDistance(robot, contact) + euclideanDistance(sensor, contact);
    
def getSimulatedDistanceReading(sensor, robot, errorStDev):
    distance = 0;
    probMultiPathTop = probSurfaceMultiPath(robot[2]);
    probMultiPathBottom = probBottomMultiPath(robot[2]);
    
    if (probMultiPathBottom >= probMultiPathTop and probMultiPathBottom >= random.random()):
        if (random.random()>0.5):
            distance = simulateMultiPath(sensor, robot, getSeaDepth());
        else:
            distance = None;
        
    elif (probMultiPathTop > probMultiPathBottom and probMultiPathTop >= random.random()):
        if (random.random()>0.5):
            distance = simulateMultiPath(sensor, robot, 0);
        else:
            distance = None;
    else:
        distance = random.gauss(euclideanDistance(sensor, robot), errorStDev);
    return distance;