#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import LaserScan
from emobile.msg import PointCloud
from numpy import pi

xVu8 = [0]*8
yVu8 = [0]*8
xM16 = [0]*16
yM16 = [0]*16

def rot(x,y,angle):
    for i in range(len(x)):
        x[i] = x[i]*cos(angle)-y[i]*sin(angle)
        y[i] = x[i]*sin(angle)+y[i]*cos(angle)
        
def translate(x,y,xoffset,yoffset):
    for i in range(len(x)):
        x[i] = x[i]+xoffset
        y[i] = y[i]+yoffset

def Vu8PointCloud(scan):
    angle = scan.angle_min
    for i,dist in enumerate(scan.ranges):
        angle += i*scan.angle_increment
        xVu8[i] = dist*cos(angle)
        yVu8[i] = dist*sin(angle)
    rot(xVu8,yVu8,-pi/2)
    translate(xVu8,yVu8)

def M16PointCloud(scan):
    angle = scan.angle_min
    for i,dist in enumerate(scan.ranges):
        angle += i*scan.angle_increment
        xM16[i] = dist*cos(angle)
        yM16[i] = dist*sin(angle)
        
        
    

# init_ros_node
rospy.init_node('leddar')
SubFrontLeddar = rospy.Subscriber('/leddarVu8', LaserScan,Vu8PointCloud)
SubFrontLeddar = rospy.Subscriber('/leddarM16', LaserScan,M16PointCloud)
pub = rospy.Publisher('/pointcloud', PointCloud,queue_size = 10)

while not rospy.is_shutdown():
    
	pub.publish(msg)