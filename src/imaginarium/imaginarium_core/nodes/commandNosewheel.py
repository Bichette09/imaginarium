#!/usr/bin/env python
# coding: utf-8

import imaginarium_core.msg
import rospy
import os
import serial

class CommandNosewheel(object):

 

if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('commandNosewheel')

	
	sRosPublisher = rospy.Publisher('imaginarium_core/CommandNosewheel', imaginarium_core.msg.CommandNosewheel, queue_size=5)
        lUltrasoundReader = Ultrasound(rospy.get_param('/commandNosewheel/serialPort'))

	while not rospy.core.is_shutdown():
	

	
		# Message publication
sRosPublisher.publish(imaginarium_core.msg.CommandNosewheel(lNoseWheelAngle))
