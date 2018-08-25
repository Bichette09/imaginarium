#!/usr/bin/env python
# coding: utf-8


import emobile.msg
import emaginarium_common.msg
import std_msgs
import rospy
import os
import math
from settings_store import settings_store_client
import time
import numpy as np

class CommandSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.L=2.
		self.D=0.75
		self.registerAttributes([
			('L','command/L','Prediction distance'),
			('D','command/D','Commanded distance from the side')
			])

class ControlLaw():
	
	def __init__(self):
		self.throttleGoal = 0
		self.steeringGoal =0
		self.ledarDist = [0]*24
		self.ledarDistX0 = [0]*24
		self.ledarDistY0 = [0]*24
		self.x_obj = 0
		self.y_obj = 0

	def updatestick(self,param):
		self.throttleGoal= param.data[3]
		#self.steeringGoal= param.data[0]
		
	def onNewLedar(self,param):
		self.ledarDist = param.distance
		self.ledarDistX0 = param.x1
		self.ledarDistY0 = param.y1

	def computeAngleToBorders(self):
		# remove null measures
		m16_X = []
		m16_Y = []
		for (x,y,d) in zip(self.ledarDistX0[8:24],self.ledarDistY0[8:24],self.ledarDist[8:24]):
			if d > 0.001:
				# ignore measures that are too close
				m16_X.append(x)
				m16_Y.append(y)
		
		lA = 0.
		lB = 0.
		lAngle=0.
		try:
			if len(m16_X) == 0:
				raise Exception()
			resFit = np.polyfit(np.array(m16_X),np.array(m16_Y),1)
			lA = resFit[0]
			lB = resFit[1]
			lAngle=math.atan(lA)*57.3
		except:
			pass
				
		if not np.isfinite(lAngle):
			return None
		return (lAngle,lA,lB)

	def computeObjectivePoint(self):
		# remove null measures
		vU8_X = []
		vU8_Y = []
		vU8_dist=[]
		i_obj=0
		nb_not_null = 0
		(lAngle,lA,lB)=self.computeAngleToBorders()
		
		for (x,y,d) in zip(self.ledarDistX0[0:8],self.ledarDistY0[0:8],self.ledarDist[0:8]):
			# ignore measures that are too close
			vU8_X.append(x)
			vU8_Y.append(y)
			vU8_dist.append(d)
			if d > 0.001:
				nb_not_null +=1
		vU8_X.append(lSettings.L)
		vU8_Y.append(lA*lSettings.L+lB+lSettings.D)
		vU8_dist.append(np.sqrt((lSettings.L)**2+(lA*lSettings.L+lB+lSettings.D)**2))
		dist_mean=np.sum(vU8_dist)/nb_not_null
		rospy.logwarn('mean '+str(dist_mean))
		indexes = [i for i,j in enumerate(vU8_dist) if (j < dist_mean and j != 0)]
		if len(vU8_X) != 0:
			for i in indexes:
				delta_mean = dist_mean-self.ledarDist[i]
				if (i-1) >0 :
					vU8_dist[i-1]= vU8_dist[i-1] - delta_mean
				if (i+1) <8:
					vU8_dist[i+1]= vU8_dist[i+1] - delta_mean
			rospy.logwarn('vu8_dist :'+str(vU8_dist))
			rospy.logwarn('ledar_dist :'+str(self.ledarDist[0:8]))
			i_obj = vU8_dist.index(max(vU8_dist))	
			rospy.logwarn('x :'+str(vU8_X)+' y :'+str(vU8_Y))
			
			self.x_obj=vU8_X[i_obj]
			self.y_obj=vU8_Y[i_obj]
			rospy.logwarn('i : '+ str(i_obj)+ ' obj :'+str(self.x_obj)+' yobj :'+str(self.y_obj))
		return lA,lB
		
		
if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('command')

	#creation instance settings pour les parametres modifiables
	lSettings = CommandSettings()
	
	sRosPublisherSteering = rospy.Publisher('emobile/CommandSteering', emobile.msg.CommandSteering, queue_size=1)
	sRosPublisherThrottle = rospy.Publisher('emobile/CommandThrottle', emobile.msg.CommandThrottle, queue_size=1)
	sRosPublisherDebug2dPrimitives = rospy.Publisher('emobile/Debug2dPrimitive', emaginarium_common.msg.Debug2dPrimitive, queue_size=1)
	
	lControlLaw = ControlLaw()
	sRosSuscriberThrottle = rospy.Subscriber('GamePadSticks', std_msgs.msg.Float32MultiArray,lControlLaw.updatestick)
	sRosSuscriberLedar = rospy.Subscriber('/pointcloud', emobile.msg.PointCloud,lControlLaw.onNewLedar)
	
	lWheelAnglec = 0.
	
	while not rospy.core.is_shutdown():
		
		# we want 100Hz reresh rate
		time.sleep(0.01)
		


		
		# Predictive command
		# Compute objective Point
		(lA,lB)=lControlLaw.computeObjectivePoint()
		#objectivePointX=lSettings.L
		#objectivePointY=lA*lSettings.L+lB+lSettings.D
		#objectivePointX=lSettings.L+lSettings.D*math.cos(lFinalAngle/57.3)
		#objectivePointY=lA*lSettings.L+lB-lSettings.D*math.sin(lFinalAngle/57.3)

		lWheelAnglec=math.atan(lControlLaw.y_obj/(lControlLaw.x_obj-0.265))*57.3
		lWheelAnglec = max(-35,min(35,lWheelAnglec))
		
		#Command normalisation
		lControlLaw.steeringGoal=lWheelAnglec/35

		sRosPublisherThrottle.publish(emobile.msg.CommandThrottle(lControlLaw.throttleGoal))
		sRosPublisherSteering.publish(emobile.msg.CommandSteering(lControlLaw.steeringGoal))
		sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('sideline','line','red',[lA,lB]))
		sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('targetPoint','circle','green',[lControlLaw.x_obj,lControlLaw.y_obj,0.05]))
		#sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('targetPoint','circle','red',[0,0,dist_mean]))
