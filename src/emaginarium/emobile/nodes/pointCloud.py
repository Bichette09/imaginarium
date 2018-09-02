#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import LaserScan
from emobile.msg import PointCloud
import numpy
from numpy import pi
import math
import warnings
warnings.simplefilter('ignore', numpy.RankWarning)

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
		
		lDistIdx = [[3,4],[2,5],[1,6],[0,7]]
		for i in range(0,4):
			for s, si in zip([-1.,1.],[0,1]):
				lIdx = lDistIdx[i][si]
				dist = scan.ranges[lIdx]
				
				if dist > 0.001:
					lAngle = 0 + s * (i + 0.5) * scan.angle_increment
					lX = dist*math.cos(lAngle)
					lY = dist*math.sin(lAngle)
					self.xVu8[lIdx] = lX + 0.0265 + 0.097;
					# dans le repère du leddar la profondeur est selon x
					# le leddar regarde selon -y dans le repère du robot
					self.yVu8[lIdx] = -lY
				else:
					self.xVu8[lIdx] = 0.
					self.yVu8[lIdx] = 0.
				self.dVu8[lIdx] = dist

		self.sendPointCloud()
		

	def M16PointCloud(self,scan):
	
		lDistIdx = [[7,8],[6,9],[5,10],[4,11],[3,12],[2,13],[1,14],[0,15]]
		for i in range(0,8):
			for s, si in zip([-1.,1.],[0,1]):
				lIdx = lDistIdx[i][si]
				dist = scan.ranges[lIdx]
				
				if dist > 0.001:
					lAngle = 0 + s * (i + 0.5) * scan.angle_increment
					lX = dist*math.cos(lAngle)
					lY = dist*math.sin(lAngle)
				
					self.xM16[lIdx] = lY + 0.09;
					# dans le repère du leddar la profondeur est selon x
					# le leddar regarde selon -y dans le repère du robot
					self.yM16[lIdx] = -lX - 0.134
				else:
					self.xM16[lIdx] = 0.
					self.yM16[lIdx] = 0.
				self.dM16[lIdx] = dist
		self.sendPointCloud()

	
if __name__ == "__main__":
	# init_ros_node
	rospy.init_node('leddar')
	
	pub = rospy.Publisher('/pointcloud', PointCloud,queue_size = 10)
	lPointCloudConverter = PointCloudConverter(pub)

	
	SubFrontLeddar = rospy.Subscriber('/leddarVu8', LaserScan,lPointCloudConverter.Vu8PointCloud)
	SubFrontLeddar = rospy.Subscriber('/leddarM16', LaserScan,lPointCloudConverter.M16PointCloud)
	
	rospy.spin() 
	
