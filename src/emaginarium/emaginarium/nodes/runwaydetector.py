#!/usr/bin/env python
# coding: utf-8

import rospy
import serial,json
import numpy as np
import emaginarium.msg
import pypot.dynamixel

# ecart entre les deux detecteurs (mm)
D = 285.0
# avance de phase (m)
avPhase = 1.5
# distance par rapport au bord voulue (m)
Obj = 0.57

wheel = 0.0

if __name__ == "__main__":
	rospy.init_node('runwaydetector')
	sRosPublisher = rospy.Publisher('emaginarium/CommandNosewheel', emaginarium.msg.CommandNosewheel, queue_size=5)
	dxl_io = pypot.dynamixel.DxlIO(rospy.get_param('/runwaydetector/serialPortDynamixel'))
	pSerialPort = serial.Serial(rospy.get_param('/runwaydetector/serialPortLidar'), baudrate=115200,timeout=1)
	
	r = rospy.Rate(50)
	while not rospy.core.is_shutdown():
		pSerialPort.write("REQ")
		out = pSerialPort.readline().replace("'",'"')
		try:
			lidars = json.loads(out)
		except Exception as e:
			print e
			lidars = {}
		#print lidars
		rightAngle = None
		rightDistance = None
		leftAngle = None
		leftDistance = None
		if len(lidars)>0:
			if lidars['L1']['S'] == 0 and lidars['L2']['S'] == 0:
				rightAngle = 180.0*np.arctan((lidars['L2']['D']-lidars['L1']['D'])/D)/np.pi
				rightDistance = (lidars['L2']['D']+lidars['L1']['D'])/2
			elif lidars['L1']['S'] > 0 and lidars['L2']['S'] == 0:
				rightDistance = lidars['L2']['D']
			elif lidars['L1']['S'] == 0 and lidars['L2']['S'] > 0:
				rightDistance = lidars['L1']['D']
				
			if lidars['L3']['S'] == 0 and lidars['L4']['S'] == 0:
				leftAngle = 180.0*np.arctan((lidars['L3']['D']-lidars['L4']['D'])/D)/np.pi
				leftDistance = (lidars['L3']['D']+lidars['L4']['D'])/2
			elif lidars['L3']['S'] > 0 and lidars['L4']['S'] == 0:
				leftDistance = lidars['L4']['D']
			elif lidars['L3']['S'] == 0 and lidars['L4']['S'] > 0:
				leftDistance = lidars['L3']['D']
			
			if leftDistance is not None and rightDistance is not None:
				Obj = (leftDistance + rightDistance-360)/2000.0
			else: 
				Obj = (1500-360)/2000.0
		
			if rightAngle is not None:
				yRight=Obj-(rightDistance/1000.0)+avPhase*np.tan(rightAngle*np.pi/180.0)
			if leftAngle is not None:
				yLeft=-Obj+(leftDistance/1000.0)+avPhase*np.tan(leftAngle*np.pi/180.0)
				
			if leftAngle is not None and rightAngle is not None:
				if leftDistance<rightDistance:
					y = yLeft
				else:
					y = yRight
			elif leftAngle is not None and rightAngle is None:
				y = yLeft
			elif leftAngle is None and rightAngle is not None:
				y = yRight
			else:
				y = 0.0
			# filter
			wheel = 0.50*wheel+0.50*(-180.0*np.arctan(y/avPhase)/np.pi)
			dxl_io.set_goal_position({1:wheel})
		r.sleep()
pSerialPort.close()
