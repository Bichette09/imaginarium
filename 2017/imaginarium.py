import SensorReader
import InputFilter
import zmq
import base64
import json
from autoThrust import autoThrust

if __name__ == '__main__':
	lAutoThrust = autoThrust(23, 24, 0.01263, 1, 0, 0)
	lMapBuilder = InputFilter.MapBuilder(SensorReader.SensorReader())
	
	context = zmq.Context()
	socket = context.socket(zmq.PUB)
	socket.bind("tcp://0.0.0.0:5556")
	
	lMapBuilder.start()
	lMap = None
	lPreviousMapRevision = 0
	while lMapBuilder.is_alive():
		#retrieve the speed
		lSpeed = lAutoThrust.speed
		#retrieve the map
		lMap = lMapBuilder.getCurrentMap(lMap);
		#convert into json
		if lPreviousMapRevision != lMap.mMapRevision:
			lPreviousMapRevision = lMap.mMapRevision
			lJson = {'hitpoints' : []}
			for lPoint in lMap.mAllPoints:
				lToPush = lPoint
				lToPush['p'] = lPoint['p'].tolist();
				lToPush['n'] = lPoint['n'].tolist();
				lJson['hitpoints'].append(lToPush)
			lMsg = b"mapviewer" + base64.b64encode(json.dumps(lJson).encode('utf-8'))
			#print('Send new map')
			socket.send(lMsg)
		lMapBuilder.join(0.05)
	
