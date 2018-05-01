#!/usr/bin/env python
# coding: utf-8

import imaginarium_core.msg
import rospy
import os
from gpiozero import Button, LED
import time

class Antenna:

	def __init__(self, pinA, pinB, pull_up=False):
		self.gpioA = Button(pinA, pull_up)
		self.gpioB = LED(pinB)
	
		self.gpioB.on()

		self.seen_impulse = false

		self.gpioA.when_pressed = lambda *args : self.pulse(self.gpioA, 1)

		self.currentTime = None

	def pulse(self, gpio, level):
		self.gpioB.off()
		self.seen_impulse = true
		self.currentTime = time.time()
		print('pulse')

	def getAndReset(self):
		if self.currentTime is not None and (time.time() - self.currentTime) > 5:
			self.gpioB.on()
			self.currentTime = None
		seen_impulse = self.seen_impulse
		self.seen_impulse = false
  		return seen_impulse 

if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('antenna')
	
	LPreviousStatus = false;
	
	sRosPublisher = rospy.Publisher('imaginarium_core/Antenna', imaginarium_core.msg.Antenna, queue_size=5)
        lAntennaInterruption = Antenna(rospy.get_param('/antenna/pinA'),rospy.get_param('/antenna/pinB'))

	while not rospy.core.is_shutdown():
	
		if lAntennaInterruption.getAndReset() is not LPreviousStatus:
		#Alors on vient de voir passer un front montant ou descendant
			# Message publication
			LPreviousStatus = not LPreviousStatus
			sRosPublisher.publish(imaginarium_core.msg.Antenna(LPreviousStatus))
		else:
			sleep(0.1)
		
