#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import LaserScan
from emobile.msg import PointCloud
from numpy import pi
import math


class PointCloudConverter():
	
	def __init__(self,pPublisher):
		self.xVu8 = [0]*8
		self.yVu8 = [0]*8
		self.dVu8 = [0]*8
		self.xM16 = [0]*16
		self.yM16 = [0]*16
		self.dM16 = [0]*16
		self.mPublisher = pPublisher
		
	def sendPointCloud(self):
		msg = PointCloud()
		
		msg.distance = self.dVu8 + self.dM16
		msg.x1 = self.xVu8 + self.xM16
		msg.y1 = self.yVu8 + self.yM16
		
		self.mPublisher.publish(msg)

	def rot(self,x,y,angle):
		for i in range(len(x)):
			x[i] = x[i]*math.cos(angle)-y[i]*math.sin(angle)
			y[i] = x[i]*math.sin(angle)+y[i]*math.cos(angle)
			
	def translate(self,x,y,xoffset,yoffset):
		for i in range(len(x)):
			x[i] = x[i]+xoffset
			y[i] = y[i]+yoffset

	def Vu8PointCloud(self,scan):
		angle = scan.angle_min
		for i,dist in enumerate(scan.ranges):
			angle += scan.angle_increment
			lX = dist*math.cos(angle)
			lY = dist*math.sin(angle)
			
			self.xVu8[i] = -lY + 0.09;
			# dans le repère du leddar la profondeur est selon x
			# le leddar regarde selon -y dans le repère du robot
			self.yVu8[i] = -lX - 0.134
			self.dVu8[i] = dist
			
		# for i in range(0,8):
			
		
		# self.translate(self.xVu8,self.yVu8,-0.134,0.09)
		#self.rot(self.xVu8,self.yVu8,-pi/2)
		#self.translate(self.xVu8,self.yVu8,0.,0.)
		self.sendPointCloud()
		

	def M16PointCloud(self,scan):
		angle = scan.angle_min
		for i,dist in enumerate(scan.ranges):
			angle += i*scan.angle_increment
			self.xM16[i] = dist*math.cos(angle)
			self.yM16[i] = dist*math.sin(angle)
			self.dM16[i] = dist
		self.sendPointCloud()

	
if __name__ == "__main__":
	# init_ros_node
	rospy.init_node('leddar')
	
	pub = rospy.Publisher('/pointcloud', PointCloud,queue_size = 10)
	lPointCloudConverter = PointCloudConverter(pub)

	
	SubFrontLeddar = rospy.Subscriber('/leddarVu8', LaserScan,lPointCloudConverter.Vu8PointCloud)
	SubFrontLeddar = rospy.Subscriber('/leddarM16', LaserScan,lPointCloudConverter.M16PointCloud)
	
	rospy.spin() 
	