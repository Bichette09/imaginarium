import pypot.dynamixel
import zmq
import time
import pigpio
import serial

#Open serial port
ser = serial.Serial('/dev/ttyACM0', timeout=10) 


def set_thrust(thrust,pi,mockup):
    # limitation
    if len(thrust)==4:
        for i in range(4):
            if thrust[i]<1000:
                thrust[i]=1000
            if thrust[i]>2000:
                thrust[i]=2000    
    if mockup == True:
        pi.set_servo_pulsewidth(26, 0)
        pi.set_servo_pulsewidth(19, 0)
        pi.set_servo_pulsewidth(13, 0)
        pi.set_servo_pulsewidth( 6, 0)
    else:
        # mapping 1->26; 2->19; 3->13 and 4->6
        pi.set_servo_pulsewidth(26, thrust[0])
        pi.set_servo_pulsewidth(19, thrust[1])
        pi.set_servo_pulsewidth(13, thrust[2])
        pi.set_servo_pulsewidth( 6, thrust[3])

#Definition du port pour le servo moteur
ports = '/dev/ttyACM1'

#Connection au servo moteur
dxl_io = pypot.dynamixel.DxlIO(ports)

IP = '0.0.0.0'
port = '8000'
dt = 0.1
lock = False
c = zmq.Context()
s = c.socket(zmq.REP)
s.bind("tcp://"+IP+":"+port)

# ouverture des GPIO
pi = pigpio.pi()
#set_thrust([1000,1000,1000,1000],pi,False) # mandatory init engines
#time.sleep(5)
thrust = 0
while True:
	reply = s.recv_json()
	if reply.has_key("pad"):
		pad = reply["pad"]
		set1000 = pad["A"]
		set2000 = pad["B"]
		if set1000:
			thrust = 1000
		elif set2000:
			thrust = 2000
		else:
			thrust = 0
		print thrust
		set_thrust([thrust,0,0,0],pi,False)
	s.send_json({})
	    
	time.sleep(dt)
