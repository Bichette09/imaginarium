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
nBeams = 16
# range (m)
rangemin = 0.0
rangemax = 15.0

# init ros node
rospy.init_node('leddar')

try:
	import minimalmodbus
except:
	rospy.logerr('Fail to import minimalmodbus, leddarM16 will not be available')
	sys.exit(0)
	
pub = rospy.Publisher('/leddarM16', LaserScan,queue_size = 1)

# init serial connection
lSerialPort = rospy.get_param('/leddarM16/serialPort')

m = minimalmodbus.Instrument(lSerialPort,1)
m.serial.baudrate = 115200
m.serial.bytesize = 8
m.serial.stopbits = 1
m.serial.timeout = 1
m.serial.parity = 'N'
m.mode = minimalmodbus.MODE_RTU


# necessary hardware pause
time.sleep(1)

lRate = rospy.Rate(42)

t=rospy.Time.now()
while not rospy.is_shutdown():
	#read the sensor
	# read register from 16 to 16+16
	t_old = t
	t=rospy.Time.now()
	lDist = m.read_registers(16,16,4)
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
	msg.intensities = [0.0]*nBeams

	# send the message
	pub.publish(msg)
	
	lRate.sleep()