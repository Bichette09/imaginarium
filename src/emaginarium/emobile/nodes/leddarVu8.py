#!/usr/bin/env python
# coding: utf-8


import time
import sys
import rospy
from sensor_msgs.msg import LaserScan
from numpy import pi


# parameters
# opening of the sensor (deg)
opening = 95
# number of beams
nBeams = 8
# range
rangemin = 0.0
rangemax = 15.0

# init ros node
rospy.init_node('leddar')

try:
	import minimalmodbus
except:
	rospy.logerr('Fail to import minimalmodbus, leddarVu8 will not be available')
	sys.exit(0)

pub = rospy.Publisher('/leddarVu8', LaserScan,queue_size = 1)

lRate = rospy.Rate(42)

# init serial connection
minimalmodbus.BAUDRATE = 115200
lSerialPort = rospy.get_param('/leddarVu8/serialPort')
m = None
try:
	m = minimalmodbus.Instrument(lSerialPort,1,'rtu')
	# necessary hardware pause
	time.sleep(1)
except:
	rospy.logerr('Fail to open com with vu8')
	
t=rospy.Time.now()
while not rospy.is_shutdown():
	if m is None:
		time.sleep(0.1)
		continue
	#read the sensor
	# read register from 16 to 16+8
	t_old = t
	t=rospy.Time.now()
	lDist = m.read_registers(16,8,4)
	for i in range(nBeams):
		lDist[i] /= 100.0

	# compose the message
	msg = LaserScan()
	# header
	msg.header.stamp = t
	msg.header.frame_id = "laser_frame"
	# start, end and increment angle of the scan (rad)
	msg.angle_min = -0.5*(opening*pi/180.0)
	msg.angle_max = 0.5*(opening*pi/180.0)
	msg.angle_increment = (opening*pi/180.0)/(nBeams-1)
	# time between measure (s)
	msg.time_increment = (t-t_old).nsecs/1000000000.0
	# max and min range (meter)
	msg.range_min = rangemin
	msg.range_max = rangemax
	# range (m)
	msg.ranges = lDist
	# intensity (?)
	msg.intensities = [0.0]*8

	# send the message
	pub.publish(msg)
	
	lRate.sleep()
