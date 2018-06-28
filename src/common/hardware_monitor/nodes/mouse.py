#!/usr/bin/env python
# coding: utf-8

import hardware_monitor.msg
import rospy
import os
import time
import std_msgs.msg
from settings_store import settings_store_client
try:
	import evdev
except:
	rospy.logerr('Mouse node need evdev module')

class MouseSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.updateperiod=0.1
		self.registerAttributes([
			('updateperiod','mouse/updateperiod','period in sec between two mouse update')
			])
		
		
if __name__ == "__main__":
	
	rospy.init_node('mouse')
	
	lDevice = None
	lEventChannel = rospy.get_param('/mouse/eventchannel')
	print(lEventChannel)
	try:
		lDevice = evdev.InputDevice(lEventChannel)
	except:
		rospy.logerr('Mouse node : fail to open ' + str(lEventChannel))
	
	lSettings = MouseSettings()
	sRosPublisher = rospy.Publisher('hardware_monitor/Mouse', hardware_monitor.msg.Mouse, queue_size=500)
	
	lSumBeforeSyncDelta = [0,0,0]
	lGotMovement = False
	
	lSyncedSum = [0,0,0]
	
	lEmitInterval = 1
	lLastEmit = time.time()
	
	while not rospy.core.is_shutdown():
		if (time.time() - lLastEmit) > lSettings.updateperiod:
			h = std_msgs.msg.Header()
			h.stamp = rospy.Time.now()
			sRosPublisher.publish(hardware_monitor.msg.Mouse(h,lSyncedSum[0],lSyncedSum[1],lSyncedSum[2]))
			lLastEmit = time.time()
			lSyncedSum = [0,0,0]
		lEvent = lDevice.read_one() if lDevice is not None else None
		if lEvent is None:
			time.sleep(0.05)
			continue

		if evdev.ecodes.EV_SYN == lEvent.type:
			if lGotMovement:
				lSyncedSum[0] = lSyncedSum[0] + lSumBeforeSyncDelta[0]
				lSyncedSum[1] = lSyncedSum[1] + lSumBeforeSyncDelta[1]
				lSyncedSum[2] = lSyncedSum[2] + lSumBeforeSyncDelta[2]
			lSumBeforeSyncDelta = [0,0,0]
			lGotMovement = False
		elif evdev.ecodes.EV_REL == lEvent.type :
			if evdev.ecodes.REL[lEvent.code] == 'REL_X':
				lSyncedSum[0] = lSyncedSum[0] + lEvent.value
				lGotMovement = True
			elif evdev.ecodes.REL[lEvent.code] == 'REL_Y':
				lSyncedSum[1] = lSyncedSum[1] + lEvent.value
				lGotMovement = True
			elif evdev.ecodes.REL[lEvent.code] == 'REL_WHEEL':
				lSyncedSum[2] = lSyncedSum[2] + lEvent.value
				lGotMovement = True
		
		