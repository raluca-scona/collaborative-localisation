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
from scipy.optimize import curve_fit

usblDepth = 2;
location = Point(0,0,0);

top = None;
usbl = None;
dvl = None;
pressure = None;
imu = None;
groundTruth = None;

usblStandardDev = 0.15;
drStandardDev = 0.1;
odometry = Point(2,2,3);

usblXData = numpy.array([]);
usblYData = numpy.array([]);
drXData = numpy.array([0.0]);
drYData = numpy.array([0.0]);
locationsX = numpy.array([]);
locationsY = numpy.array([]);
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
    
def initLocation(initPoint):
    location = Point(initPoint.x + random.uniform(-3,3), initPoint.y + random.uniform(-3,3), initPoint.z)
    return location;

def updateMotion(location, dvl, imu, timeDifference):
    #print 'here'
    newLocation = Point(0,0, location.z);
    global odometry;
    global drXData;
    global drYData;
    
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
    newLocation.x = location.x + deadReckoning.item(0,0);
    newLocation.y = location.y + deadReckoning.item(1,0);
    
    drXData[len(drXData) - 1] += deadReckoning.item(0,0);
    drYData[len(drYData) - 1] += deadReckoning.item(1,0);
        
    return newLocation;
    
def getMedianPoint(usblPoints):
    distances = usblPoints.keys();
    distances.sort();
    return usblPoints[distances[3]];

def gradientDescent(usblX, usblY, drX, drY, locationsX, locationsY):
    
    tempLocationsX = numpy.zeros(len(locationsX));
    tempLocationsY = numpy.zeros(len(locationsY));
    numpy.copyto(tempLocationsX, locationsX);
    numpy.copyto(tempLocationsY, locationsY);

    epsilon = 0.001;
    oldTempLocationsX = numpy.zeros(len(locationsX));
    oldTempLocationsY = numpy.zeros(len(locationsY));
    
    alpha = 0.01;
    iters = 0;
    while(math.sqrt(euclideanDistance(tempLocationsX, oldTempLocationsX)**2 + euclideanDistance(tempLocationsY, oldTempLocationsY)**2) >= epsilon):
        iters= iters+1;
        numpy.copyto(oldTempLocationsX, tempLocationsX);
        numpy.copyto(oldTempLocationsY, tempLocationsY);

        for i in range(len(locationsX)):
            tempLocationsX[i] = tempLocationsX[i] - alpha*( (locationsX[i] - usblX[i])/usblStandardDev );
            tempLocationsY[i] = tempLocationsY[i] - alpha*( (locationsY[i] - usblY[i])/usblStandardDev );
            if (len(locationsX) > 1):
                if (i == 0):
                    tempLocationsX[i] = tempLocationsX[i] - alpha* ( -(locationsX[i+1] - locationsX[i] - drX[i+1])/drStandardDev );
                    tempLocationsY[i] = tempLocationsY[i] - alpha* ( -(locationsY[i+1] - locationsY[i] - drY[i+1])/drStandardDev);
                elif (i > 0 and i < len(locationsX) - 1):
                        tempLocationsX[i] = tempLocationsX[i] - alpha*( (locationsX[i] - locationsX[i-1] - drX[i])/drStandardDev - (locationsX[i+1] - locationsX[i] - drX[i+1])/drStandardDev)
                        tempLocationsY[i] = tempLocationsY[i] - alpha*( (locationsY[i] - locationsY[i-1] - drY[i])/drStandardDev - (locationsY[i+1] - locationsY[i] - drY[i+1])/drStandardDev)
                else:
                    tempLocationsX[i] = tempLocationsX[i] - alpha*( (locationsX[i] - locationsX[i-1] - drX[i])/drStandardDev );
                    tempLocationsY[i] = tempLocationsY[i] - alpha*( (locationsY[i] - locationsY[i-1] - drY[i])/drStandardDev);
        numpy.copyto(locationsX, tempLocationsX);
        numpy.copyto(locationsY, tempLocationsY);
    print iters;
    return [locationsX, locationsY];
    

location = initLocation(Point(2,2,3));
  
def runNLS():
    dvlTime = time.time();
    startTime = time.time();
    #nlsdata = open('nls.txt', 'w');
    #usbldata = open('usbl.txt', 'w');
    #odometrydata = open('odometry.txt', 'w');
    #truthdata = open('truth.txt', 'w');
    nlserrors = open('nlserr.txt', 'w');

    global location;
    global groundTruth;
    global imu;
    global dvl;
    global top;
    global usbl;
    global drXData;
    global drYData;
    global usblXData;
    global usblYData;
    global locationsX;
    global locationsY;
    global usblPoints;
    
    pub = rospy.Publisher('/uwsim/NLS_AZOR', Point, queue_size=10)
    rospy.init_node('NLS_AZOR', anonymous=True)
    
    rospy.Subscriber("/uwsim/GPS_TOP", NavSatFix, setTop)
    rospy.Subscriber("/uwsim/usbl_AZOR", USBL, setUSBL)
    rospy.Subscriber("/uwsim/DVL_AZOR", DVL, setDVL)
    rospy.Subscriber("/uwsim/PRESSURESENSOR_AZOR", Pressure, setPressure)
    rospy.Subscriber("/uwsim/IMU_AZOR", Imu, setIMU)
    rospy.Subscriber("/uwsim/odom_AZOR", Odometry, setGroundTruth)
    
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pointFromUSBL = None;
       
        if(pressure):
            location.z = pressure.pressure
            
        if(top and usbl and usbl.distance.distance and usbl.bearing.bearing):
            topPoint = [top.latitude, top.longitude, 2];
            pointFromUSBL = getPointFromUSBLReading(topPoint, usbl.bearing.bearing, usbl.distance.distance, location.z)
            #usblPoints[usbl.distance.distance] = pointFromUSBL;
           
            top = None;
            usbl = None;
            #pointFromUSBL = None;
        
        if(dvl and imu):
            location = updateMotion(location, dvl, imu, time.time() - dvlTime);
            dvlTime = time.time();
            
            if (pointFromUSBL):
                print 'got usbl point'
                #pointFromUSBL = getMedianPoint(usblPoints);
                
                usblXData = numpy.append(usblXData, pointFromUSBL.x); 
                usblYData = numpy.append(usblYData, pointFromUSBL.y);

                locationsX = numpy.append(locationsX, location.x);
                locationsY = numpy.append(locationsY, location.y);
                
                [locationsX, locationsY] = gradientDescent(usblXData, usblYData, drXData, drYData , locationsX, locationsY);
                location.x = locationsX[len(locationsX) - 1]
                location.y = locationsY[len(locationsY) - 1]
                drXData = numpy.append(drXData, 0.0);
                drYData = numpy.append(drYData, 0.0);
                location = updateMotion(location, dvl, imu, time.time() - dvlTime);
                dvlTime = time.time();
                

                '''print time.time() - startTime
                if (time.time() - startTime < 200):
                    averageLocation = Point(numpy.mean(location[0]), numpy.mean(location[1]), numpy.mean(location[2]))
                    nlsdata.write(`averageLocation.x` + " " + `averageLocation.y` + " " + `averageLocation.z` + "\n")
                    odometrydata.write(`odometry.x` + " " + `odometry.y` + " " + `odometry.z`+ "\n")
                    pos = groundTruth.pose.pose.position;
                    truthdata.write(`pos.x` + " " + `pos.y` + " " + `pos.z`+ "\n");
                    usbldata.write(`pointFromUSBL.x` + " " + `pointFromUSBL.y` + " " +`pointFromUSBL.z`+ "\n");
                else:
                    nlsdata.close();
                    odometrydata.close();
                    truthdata.close();
                    usbldata.close();'''
                
                pointFromUSBL = None;
                
            dvl = None;
            imu = None;
       
        #averageParticle = computeAverageParticle(particles, probabilities);
        
        if (groundTruth):
            pos = groundTruth.pose.pose.position
            error = math.sqrt((location.x - pos.x)**2 +  (location.y - pos.y)**2+(location.z- pos.z)**2 );
            nlserrors.write(`error` + "\n");
            print"%.2f" % location.x, "%.2f" % location.y, "%.2f" % location.z, "%.3f" %  error;
            print"%.2f" % pos.x, "%.2f" % pos.y, "%.2f" % pos.z;

        pub.publish(location);
        
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        runNLS()
    except rospy.ROSInterruptException:
        pass