#!/usr/bin/env python
# coding: utf-8

# import grading_machine.srv
# import grading_machine.msg
from settings_store import settings_store_client
import rospy
import sys
import codecs
import json
import os
from sensor_msgs.msg import CompressedImage
import picamera
import picamera.array
import cv2
import numpy
import time
import io

class GradingSettings(settings_store_client.SettingsBase):
	
	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.mUpdatePeriod = 1.
		self.mMessage = 'yopyop'
		
		self.registerAttributes([
			('mUpdatePeriod','GradingSettings/UpdatePeriod',0.01,10),
			('mMessage','GradingSettings/Message'),
			])
		

def how_long(start, op):
	print('%s took %.2fs' % (op, time.time() - start))
	return time.time()
	
if __name__ == "__main__":
	rospy.init_node('grading_machine_test')
	
	
	lSettings = GradingSettings()
	
	lImgPub = image_pub = rospy.Publisher("/test/image_raw/compressed",CompressedImage, queue_size = 1)
	camera = picamera.PiCamera()
	# camera.resolution = (640,480)
	# camera.framerate=90
	# lCounter = 0
	rawCapture = picamera.array.PiRGBArray(camera)
	yuvCapture = picamera.array.PiYUVArray(camera)
	
	# rospy.logwarn(lSettings.mMessage + str(lCounter))
	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(lSettings.mUpdatePeriod)
		rospy.logwarn(lSettings.mMessage)
		stream = io.BytesIO()
		# rospy.rostime.wallsleep(2.)
		start = time.time()
		camera.capture(stream, format='jpeg', use_video_port=True)
		# camera.capture('out.jpg')
		# yuvCapture.truncate(0)
		# camera.capture(yuvCapture, format="yuv")
		how_long(start,'capture yuv')
		
		# rospy.rostime.wallsleep(2.)
		# start = time.time();
		# rawCapture.truncate(0)
		# camera.capture(rawCapture, format="bgr")
		# how_long(start,'capture rgb')
		# image = rawCapture.array
		image = stream
		
		# lCounter = lCounter + 1
		# if (lCounter % 15) == 0:
			# rospy.logwarn(lSettings.mMessage + str(lCounter))
		
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		# msg.data = numpy.array(cv2.imencode('.jpg', image)[1]).tostring()
		msg.data = stream.getvalue()
		lImgPub.publish(msg)
		
	