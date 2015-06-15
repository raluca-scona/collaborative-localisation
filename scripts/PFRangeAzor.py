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

totalParticles = 500;
usblDepth = 2;
particles = [];
probabilities = [];
top = None;
usbl = None;
dvl = None;
pressure = None;
imu = None;
groundTruth = None;
usblStandardDev = 0.15;
odometry = Point(2,2,3);
usblPoints = {};


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
    
def initParticles(initPoint, minRange, maxRange):
    particles = [];
    
    for i in range(totalParticles):
        r = -1;
        while (r<minRange):
            r = math.sqrt(random.random())*maxRange;
        theta = random.random()*2*math.pi;
        
        particle = Point(initPoint.x + r*math.cos(theta), initPoint.y + r*math.sin(theta), initPoint.z)  
        particles.append(particle);
    return particles;
    

def updateMotion(particles, dvl, imu, timeDifference):
    newParticles = [];
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
    
    for i in range(len(particles)):
        newParticle = Point(particles[i].x, particles[i].y, particles[i].z);
        newParticle.x += deadReckoning.item(0,0) + random.gauss(0, 0.1);
        newParticle.y += deadReckoning.item(1,0) + random.gauss(0, 0.1);
        newParticles.append(newParticle);
        
    return newParticles;

def assignProbabilitiesToParticles(particles, topPoint, rangeReading):
    probabilities = [];
    for p in particles:
        distance = math.sqrt( (p.x - topPoint.x)**2 + (p.y - topPoint.y)**2 + (p.z - topPoint.z)**2)    
        prob = 1. /(usblStandardDev * math.sqrt( 2 * math.pi)) * math.e ** ( - (distance - rangeReading)**2/(2*usblStandardDev**2))
        probabilities.append(prob)
    return probabilities;

def normaliseProbabilities(probabilities):
    normalisationFactor = sum(probabilities)
    normalisedProbabilities = [probability/normalisationFactor for probability in probabilities]
    return normalisedProbabilities
    
def resampleParticles(particles, probabilities):
    newParticles = []
    
    cummulatedProbability = [0] * len(probabilities);
    cummulatedProbability[0] = probabilities[0];
    
    for i in range(1, len(probabilities)):
        cummulatedProbability[i] = cummulatedProbability[i-1] + probabilities[i]
        
    for i in range(len(particles)):
        threshold = random.random();
        for j in range(len(particles)):
            if (cummulatedProbability[j] >= threshold):
                newParticles.append(particles[j]);
                break;
                
    return newParticles;

def computeAverageParticle(particles, probabilities):
    averageParticle = Point(0,0,0);
    for i in range(len(particles)):
        averageParticle.x += particles[i].x * probabilities[i];
        averageParticle.y += particles[i].y * probabilities[i];
        averageParticle.z += particles[i].z * probabilities[i];
    return averageParticle
    
def getMedianPoint(usblPoints):
    distances = usblPoints.keys();
    distances.sort();
    return usblPoints[distances[3]];

particles = initParticles(Point(2,2,3), 0, 1);
probabilities = numpy.ones(totalParticles)/totalParticles; 

  
def runParticleFilter():
    dvlTime = time.time();
    pferrors = open('pferr.txt', 'w');

    global particles;
    global probabilities;
    global groundTruth;
    global imu;
    global dvl;
    global top;
    global usbl;
    global usblPoints;
    
    pub = rospy.Publisher('/uwsim/PFRANGE_AZOR', Point, queue_size=10)
    rospy.init_node('PFRANGE_AZOR', anonymous=True)
    
    rospy.Subscriber("/uwsim/GPS_TOP", NavSatFix, setTop)
    rospy.Subscriber("/uwsim/usbl_AZOR", USBL, setUSBL)
    rospy.Subscriber("/uwsim/DVL_AZOR", DVL, setDVL)
    rospy.Subscriber("/uwsim/PRESSURESENSOR_AZOR", Pressure, setPressure)
    rospy.Subscriber("/uwsim/IMU_AZOR", Imu, setIMU)
    rospy.Subscriber("/uwsim/odom_AZOR", Odometry, setGroundTruth)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rangeReading = None;
        topPoint = None;
       
        if(pressure):
             for p in particles:            
                 p.z = pressure.pressure + random.gauss(0, 0.02);
                        
        if(top and usbl and usbl.distance.distance and usbl.bearing.bearing):
            topPoint = Point(top.latitude, top.longitude, 2)
            rangeReading =  usbl.distance.distance
            #usblPoints[usbl.distance.distance] = pointFromUSBL;
            #pointFromUSBL = None;
            top = None;
            usbl = None;
        
        if(dvl and imu):
            newTime = time.time();
            particles = updateMotion(particles, dvl, imu, newTime - dvlTime);
            dvlTime = newTime;
                       
            if (topPoint and rangeReading):
                #pointFromUSBL = getMedianPoint(usblPoints);
                #print "usbl reading ", pointFromUSBL.x, pointFromUSBL.y, pointFromUSBL.z;
                probabilities = assignProbabilitiesToParticles(particles, topPoint, rangeReading);
                print  sum(probabilities)
                if (sum(probabilities) == 0):
                    particles = initParticles(topPoint, max(0, rangeReading-2), rangeReading+2);
                    probabilities = numpy.ones(totalParticles)/totalParticles;                      
                
                probabilities = normaliseProbabilities(probabilities);
                particles = resampleParticles(particles, probabilities);

                rangeReading = None;
                topPoint = None;
            
            dvl = None;
            imu = None;
       
        averageParticle = computeAverageParticle(particles, probabilities);
        
        if (groundTruth):
            pos = groundTruth.pose.pose.position
            error = math.sqrt((averageParticle.x - pos.x)**2 +  (averageParticle.y - pos.y)**2+(averageParticle.z - pos.z)**2 );
            print pos.x, pos.y, pos.z, error;
            pferrors.write(`error` + "\n");


        pub.publish(averageParticle)
        
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        runParticleFilter()
    except rospy.ROSInterruptException:
        pass