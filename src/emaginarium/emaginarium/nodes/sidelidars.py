#!/usr/bin/env python
# coding: utf-8

import rospy
import serial,json
import numpy as np
import emaginarium.msg

'''table des etats LIDARS
0 : range valid
1 : sigma estimator check is above the internal defined threshold
2 : signal value is below the internal defined threshold
3 : Target is below minimum detection threshold
4 : phase is out of bounds
5 : HW or VCSEL failure
6 : The Range is valid but the wraparound check has not been done
7 : Wrapped target, not matching phases
8 : Internal algo underflow or overflow in lite ranging
9 : Specific to lite ranging
10 : 1st interrupt when starting ranging in back to back mode. Ignore data
11 : All Range ok but object is result of multiple pulses merging together
12 : Used by RQL as different to phase fail
13 : Target is below minimum detection threshold
14 : The reported range is invalid
'''

class SideLidars(object):
	def __init__(self,serialPortName,pub):
		# ecart entre les deux detecteurs (mm)
		self._D = 285.0
		self.refreshRate = 50
		self._pSerialPort = serial.Serial(serialPortName, baudrate=115200,timeout=1)
		self._publisher = pub
		
	def execute(self):
		self._pSerialPort.write('A')
		out = self._pSerialPort.readline()
		try:
			lidars = json.loads(out)
		except Exception as e:
			print e
			print out
			lidars = {}
		msg = emaginarium.msg.Sides()
		if len(lidars)>0:
			if lidars['L1']['S'] == 0 and lidars['L2']['S'] == 0:
				msg.rightAngle = 180.0*np.arctan((lidars['L2']['D']-lidars['L1']['D'])/self._D)/np.pi
				msg.rightDistance = (lidars['L2']['D']+lidars['L1']['D'])/2000.0
				msg.rightAngleValid = True
				msg.rightDistanceValid = True
			elif lidars['L1']['S'] > 0 and lidars['L2']['S'] == 0:
				msg.rightDistance = lidars['L2']['D']/1000.0
				msg.rightAngleValid = False
				msg.rightDistanceValid = True
			elif lidars['L1']['S'] == 0 and lidars['L2']['S'] > 0:
				msg.rightDistance = lidars['L1']['D']/1000.0
				msg.rightAngleValid = False
				msg.rightDistanceValid = True
				
			if lidars['L3']['S'] == 0 and lidars['L4']['S'] == 0:
				msg.leftAngle = 180.0*np.arctan((lidars['L3']['D']-lidars['L4']['D'])/self._D)/np.pi
				msg.leftDistance = (lidars['L3']['D']+lidars['L4']['D'])/2000.0
				msg.leftAngleValid = True
				msg.leftDistanceValid = True
			elif lidars['L3']['S'] > 0 and lidars['L4']['S'] == 0:
				msg.leftDistance = lidars['L4']['D']/1000.0
				msg.leftAngleValid = False
				msg.leftDistanceValid = True
			elif lidars['L3']['S'] == 0 and lidars['L4']['S'] > 0:
				msg.leftDistance = lidars['L3']['D']/1000.0
				msg.leftAngleValid = False
				msg.leftDistanceValid = True
			self._publisher.publish(msg)
				
	def close(self):
		self._pSerialPort.close()

if __name__ == "__main__":
	rospy.init_node('sidelidars')
	sSidePublisher = rospy.Publisher('emaginarium/Sides', emaginarium.msg.Sides, queue_size=5)
	sid = SideLidars(rospy.get_param('/sidelidars/serialPortLidar'),sSidePublisher)
	rospy.sleep(5)
	r = rospy.Rate(sid.refreshRate)
	while not rospy.core.is_shutdown():
		sid.execute()
		r.sleep()
sid.close()
