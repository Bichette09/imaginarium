from rot import RotaryEncoder
import time
import threading

class autoThrust:
	
	def __init__(self, pinA, pinB, MPerStep, Kp, Ki, Kd, pRunThread = True):
		self.count = 0
		self.dist = 0.0
		self.speed = 0.0
		self.goalSpeed = 0.0
		self.Thrust = 0.0
		self._MPerStep = MPerStep
		self._Kp = Kp
		self._Ki = Ki
		self._Kd = Kd
		self._active = 1
		self._DT = 0.1
		self._t = time.time()
		self._c = self.count
		self._rotaryEncoder = RotaryEncoder(pinA,pinB)
		self._rotaryEncoder.when_rotated = self.change
		self._thread = threading.Thread(target=self.run)
		self._thread.start()
	
	def change(self,value):
		self.count = self.count + value
		
	def run(self):
		error = 0.0
		ierror = 0.0
		while self._active:
			t_old = self._t
			c_old = self._c
			self._t = time.time()
			self._c = self.count
			
			# compute delta
			dc = self._c - c_old
			dt = self._t - t_old
			
			# compute length
			dl = dc*self._MPerStep
			self.dist = self.dist+dl
			
			# compute speed
			self.speed = dl/dt
			print self.speed
			
			# compute thrust (PID)
			error_old = error
			ierror_old = ierror
			error = self.speed-self.goalSpeed
			derror = (error-error_old)/dt
			ierror = ierror_old+(error*dt)
			if self._Ki>0:
				if ierror>(1-self._Kp*error-self._Kd*derror)/self._Ki:
					ierror = (1-self._Kp*error-self._Kd*derror)/self._Ki
				if ierror<(-1-self._Kp*error-self._Kd*derror)/self._Ki:
					ierror = (-1-self._Kp*error-self._Kd*derror)/self._Ki
			self.Thrust = self._Kp*error+self._Ki*ierror+self._Kd*derror
			
			while (time.time()-self._t)<self._DT:
				time.sleep(0.001)
	def close(self):
		self._active = 0
		
if __name__ == '__main__':
	at = autoThrust(24,23,0.01263,1,0,0)
	time.sleep(60)
	at.close()
	
	
	
	
