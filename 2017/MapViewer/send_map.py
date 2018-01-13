import zmq
import random
import sys
import time
import json

if len(sys.argv) != 2:
	print('need input file')
	sys.exit(0)

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")

with open(sys.argv[1], 'r') as f:
	for r in f:
		# print(r)
		socket.send(b"mapviewer" + r.encode('utf-8'))
		print(r)
		time.sleep(0.1)

# while True:
	# topic = random.randrange(9999,10005)
	# messagedata = random.randrange(1,215) - 80
	# lDict = {"a":topic,"b":messagedata}
	# lToSend = json.dumps(lDict).encode('utf-8')
	# print(lToSend)
	# socket.send(lToSend)
	# time.sleep(1)