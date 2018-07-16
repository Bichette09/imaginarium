#!/usr/bin/env python
# coding: utf-8

from settings_store import settings_store_client
import rospy
import sys
import os
from sensor_msgs.msg import CompressedImage
import picamera
import time
import io

class PiCameraSettings(settings_store_client.SettingsBase):
	
	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.mUpdatePeriod = 0.2
		self.mHighQuality = False
		self.mHFlip = False
		self.mVFlip = False
		self.registerAttributes([
			('mUpdatePeriod','picamera_capture/UpdatePeriod',0.01,10),
			('mHighQuality','picamera_capture/HighQuality'),
			('mHFlip','picamera_capture/HFlip'),
			('mVFlip','picamera_capture/VFlip'),
			])
	
if __name__ == "__main__":
	rospy.init_node('picamera_capture')
	
	lSettings = PiCameraSettings()
	
	lImgPub = rospy.Publisher('/picamera_capture/image_raw/compressed',CompressedImage, queue_size = 1)
	
	lNewRes = (int(rospy.get_param('/picamera_capture/width')),int(rospy.get_param('/picamera_capture/height')))
	if lNewRes[0] <= 0 and lNewRes[1] <= 0 :
		lNewRes = None
	
	lCamera = picamera.PiCamera(resolution=lNewRes, framerate=25) 
	
	while not rospy.core.is_shutdown():
		lStart = time.time()

		lStream = io.BytesIO()
		lCamera.hflip = lSettings.mHFlip
		lCamera.vflip = lSettings.mVFlip
		lCamera.capture(lStream, format='jpeg', use_video_port = not lSettings.mHighQuality)
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = lStream.getvalue()
		lImgPub.publish(msg)
		
		lStop = time.time();
		lDelta = (lStop - lStart)
		
		lDelta = lSettings.mUpdatePeriod - lDelta
		if lDelta > 0.:
			rospy.rostime.wallsleep(lDelta)
