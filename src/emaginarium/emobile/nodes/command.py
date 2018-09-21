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
# import scipy.stats

class CommandSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.L=2.
		self.D=0.75
		self.distMin=0.10
		self.distMoy=0.50
		self.distLatMin = 0.50
		self.distLatMinMin = 0.30
		self.distLatMax = 0.75
		self.distLatMaxMax = 1.0
		self.registerAttributes([
			('L','command/L','Prediction distance'),
			('D','command/D','Commanded distance from the side'),
			('distMin','command/distMin','Distance minimum before stop'),
			('distMoy','command/distMoy','Distance medium for slow speed'),
			('distLatMin','command/distLatMin','Lateral distance minimum'),
			('distLatMinMin','command/distLatMinmin','Lateral distance minimum avant contact'),
			('distLatMax','command/distLatMax','Lateral distance maximum'),
			('distLatMaxMax','command/distLatMaxMax','Lateral distance maximum du desespoir')
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
		self.coefADroite = 0
		self.coefBDroite = 0
		self.pingLidarDist = 0
		self.mFrontLedarPos = np.array([0.265 + 0.097,0.])

	def computeThrottleObjective(self):
		
		vu8_dist=[j for i,j in enumerate(self.ledarDist[0:8]) if j != 0]
		minDist=0
		if len(vu8_dist) != 0:
			minDist = min(vu8_dist)
			if minDist  > lSettings.distMoy :
				self.throttleGoal =1
			elif minDist <= lSettings.distMoy and minDist > lSettings.distMin:
				self.throttleGoal = 0.5
			else:
				self.throttleGoal =0
		else:
			self.throttleGoal =0
		sRosPublisherDebugMinDist.publish(std_msgs.msg.Float32(minDist))
		
	def onNewLedar(self,param):
		self.ledarDist = param.distance
		self.ledarDistX0 = param.x1
		self.ledarDistY0 = param.y1

	def pingLidar(self,param):
		self.pingLidarDist = param.data

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
			
			# lA, lB, r_value, p_value, std_err = scipy.stats.linregress(m16_X, m16_Y)
			
			# if r_value < 0.6:
				# lA
			
			resFit = np.polyfit(np.array(m16_X),np.array(m16_Y),1)
			lA = resFit[0]
			lB = resFit[1]
			
			lAngle=math.atan(lA)*57.3
		except:
			pass
				
		if not np.isfinite(lAngle):
			return None

		self.coefADroite=lA
		self.coefBDroite=lB

		return (lAngle)
	
	def computeAttractivePoint(self):
	# la fonction calcule les coordonnees x,y et d du point attractif en fonction des settings donnes
		(lAngle)=self.computeAngleToBorders()
		lXattractive = lSettings.L
		lYattractive = self.coefADroite*lSettings.L+self.coefBDroite+lSettings.D
		
		# compute distance to front lidar so this distance could be compared to front lidar dists
		lDattractive = np.linalg.norm(np.array([lXattractive,lYattractive]) - self.mFrontLedarPos)

		return (lXattractive, lYattractive,lDattractive)

	def findIndexAttractive(self,lYlist,lYattrac):
		lindexAttract = 0

		# On cherche tous les index qui ont un y superieur a celui du point attractif
		lindexes = [i for i,j in enumerate(lYlist) if (j <= lYattrac and j != 0)]

		# On prend le plus petit index (on suppose que les points sont ranges de maniere croissante)	
		if len(lindexes) != 0:
			lindexAttract = min(lindexes)
			
		return (lindexAttract)

	def computeObjectivePoint(self):
		# remove null measures
		vU8_X = []
		vU8_Y = []
		vU8_dist=[]
		i_obj=0
		nb_not_null = 0
		(lXattractive, lYattractive, lDattractive)=self.computeAttractivePoint()
		
		for (x,y,d) in zip(self.ledarDistX0[0:8],self.ledarDistY0[0:8],self.ledarDist[0:8]):
			# ignore measures that are too close
			vU8_X.append(x)
			vU8_Y.append(y)
			vU8_dist.append(d)
			if d > 0.001:
				nb_not_null +=1

		# Calcul de la distance moyenne des points
		dist_mean= np.sum(vU8_dist)/nb_not_null if nb_not_null > 0 else 0

		minDist = min(vU8_dist[3],vU8_dist[4])
		if minDist > lSettings.distMoy:
			# On modifie les listes pour rajouter le point attractif
			(lindexAttract)=self.findIndexAttractive(vU8_Y,lYattractive)
			vU8_X.insert(lindexAttract,lXattractive)
			vU8_Y.insert(lindexAttract,lYattractive)
			vU8_dist.insert(lindexAttract,lDattractive)
		else:
			lXattractive=0
			lYattractive=0

		#rospy.logwarn('x :'+str(vU8_X)+' y :'+str(vU8_Y))

		indexes = [i for i,j in enumerate(vU8_dist) if (j < dist_mean and j != 0)]
		if len(vU8_X) != 0:
			for i in indexes:
				delta_mean = dist_mean-self.ledarDist[i]
				if (i-1) >0 :
					vU8_dist[i-1]= vU8_dist[i-1] - delta_mean
				if (i+1) <len(vU8_X):
					vU8_dist[i+1]= vU8_dist[i+1] - delta_mean

			# Penalisation a partir des points du M16
			minYO = abs(self.coefADroite * 0.09 + self.coefBDroite)
			if self.pingLidarDist < lSettings.distLatMin:
				vU8_dist[0]=0
				vU8_dist[1]=0

			if self.pingLidarDist < lSettings.distLatMinMin:
				vU8_dist[2]=0

			if minYO < lSettings.distLatMin:
				vU8_dist[-1]=0
 				vU8_dist[-2]=0
			if minYO < lSettings.distLatMinMin:
				vU8_dist[-3]=0
				
			# Calcul du point le plus eloigne
			i_obj = vU8_dist.index(max(vU8_dist))	

			self.x_obj=vU8_X[i_obj]
			self.y_obj=vU8_Y[i_obj]
		
		# compute position of virtual points
		lVirtualPoints = []
		for (x,y,d) in zip(vU8_X,vU8_Y,vU8_dist):
			# compute dir vector
			lDir = np.array([x,y]) - self.mFrontLedarPos
			lLen = np.linalg.norm(lDir)
			if lLen <= 0.:
				continue
			lDir = lDir / lLen
			lVirtualPoint = self.mFrontLedarPos + d*lDir
			lVirtualPoints.append(lVirtualPoint[0])
			lVirtualPoints.append(lVirtualPoint[1])
			lVirtualPoints.append(0.05)
		sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('virtualpoints','circle','rgba(0,0,255,0.4)',lVirtualPoints))
		
		sRosPublisherDebugLineDist.publish(std_msgs.msg.Float32(minYO))
		return (lXattractive,lYattractive,dist_mean)
		
if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('command')

	#creation instance settings pour les parametres modifiables
	lSettings = CommandSettings()
	
	sRosPublisherSteering = rospy.Publisher('emobile/CommandSteering', emobile.msg.CommandSteering, queue_size=1)
	sRosPublisherThrottle = rospy.Publisher('emobile/CommandThrottle', emobile.msg.CommandThrottle, queue_size=1)
	sRosPublisherDebugLineDist = rospy.Publisher('emobile/DebugLineDist', std_msgs.msg.Float32, queue_size=1)
	sRosPublisherDebugMinDist = rospy.Publisher('emobile/DebugMinDist', std_msgs.msg.Float32, queue_size=1)
	sRosPublisherDebug2dPrimitives = rospy.Publisher('emobile/Debug2dPrimitive', emaginarium_common.msg.Debug2dPrimitive, queue_size=1)
	
	lControlLaw = ControlLaw()
	sRosSuscriberLedar = rospy.Subscriber('/pointcloud', emobile.msg.PointCloud,lControlLaw.onNewLedar)
	sRosSuscriberLidar = rospy.Subscriber('/emobile/PingLidarDist', std_msgs.msg.Float32,lControlLaw.pingLidar)
	
	lWheelAnglec = 0.
	
	lRate = rospy.Rate(30)
	
	while not rospy.core.is_shutdown():
		
		lRate.sleep()
		
		# Predictive command
		# Compute objective Point
		(lxAt,lyAt,dist_mean)=lControlLaw.computeObjectivePoint()

		#Compute Wheel angle 
		lWheelAnglec=math.atan(lControlLaw.y_obj/(lControlLaw.x_obj-0.265))*57.3
		lWheelAnglec = max(-35,min(35,lWheelAnglec))
		
		#Command normalisation
		lControlLaw.steeringGoal=lWheelAnglec/35

		#Compute Speed goal
		lControlLaw.computeThrottleObjective()


		sRosPublisherThrottle.publish(emobile.msg.CommandThrottle(lControlLaw.throttleGoal))
		sRosPublisherSteering.publish(emobile.msg.CommandSteering(lControlLaw.steeringGoal))
		sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('sideline','line','red',[lControlLaw.coefADroite,lControlLaw.coefBDroite]))
		sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('targetPoint','circle','green',[lControlLaw.x_obj,lControlLaw.y_obj,0.05]))
		sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('targetPoint2','circle','red',[lxAt,lyAt,0.05]))
		sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('moy','circle','rgba(0,0,255,0.4)',[0.362,0,dist_mean]))
