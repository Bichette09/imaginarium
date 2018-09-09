#!/usr/bin/env python
# coding: utf-8

import rospy
from emobile.msg import Speed
from numpy import pi
from gpiozero import Button

class Odometer(object):
	def __init__(self,pin,pub):
		self.lRate = rospy.Rate(10)
		self.lCount = 0
		self.lKMetersByTick = 2*pi*0.034/16
		self.lButton = Button(pin,pull_up=True)
		self.lButton.when_pressed = self.tick()
		self.lButton.when_released = self.tick()
		self.lLastTick = rospy.get_time()
		self.lPub = pub
		
	def tick(self):
		self.lCount +=1
		
	def sendSpeed(self):
		t = rospy.get_time()
		tCount = self.lCount
		self.lCount = 0
		msg = Speed()
		msg.speed = tCount*self.lKMetersByTick/(t-self.lLastTick)
		self.lLastTick = t
		self.lPub.Publish(msg)

if __name__ == '__main__':
	# init ros node
	rospy.init_node('odometer')
	pub = rospy.Publisher('/speed', Speed,queue_size = 1)
	odo = Odometer(rospy.get_param('/odometer/pin',pub)
	while not rospy.is_shutdown():
		odo.sendSpeed()
		odo.lRate.sleep()