#!/usr/bin/env python
# coding: utf-8

# import grading_machine.srv
# import grading_machine.msg
from settings_store import settings_store_client
import rospy
import sys
from grading_machine import oled_display_client
import cv2
from sensor_msgs.msg import CompressedImage

class GradingSettings(settings_store_client.SettingsBase):
	
	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		# self.mUpdatePeriod = 1.
		self.mMessage = 'yopyop'
		
		self.registerAttributes([
			# ('mUpdatePeriod','GradingSettings/UpdatePeriod',0.01,10),
			('mMessage','GradingSettings/Message'),
			])
		
	
if __name__ == "__main__":
	rospy.init_node('grading_machine_test')
	
	img = cv2.imread('/home/pi/image.jpg')
	img = cv2.resize(img, (0,0), fx=0.1, fy=0.1) 
	# img = cv2.resize(img, (1920,1080)) 
	img = cv2.resize(img, (960,540)) 
	lImgPub = rospy.Publisher('/test/image_raw/compressed',CompressedImage, queue_size = 1)
	
	
	lSettings = GradingSettings()

	lDisp = oled_display_client.Display()
	rospy.logwarn('Test ready')
	lDisp.drawRect(0,0,15,15)
	lTextArea = oled_display_client.TextArea(lDisp,[0,0,128,96])
	
	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.01)
		lTextArea.drawText(lSettings.mMessage)
		
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = cv2.imencode('.jpg', img)[1].tostring()
		lImgPub.publish(msg)
		# rospy.logwarn('top')
	