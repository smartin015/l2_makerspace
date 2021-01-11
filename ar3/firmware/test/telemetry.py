import zmq
import time
import struct

context = zmq.Context()

#  Socket to talk to server
print("Connecting...")
socket = context.socket(zmq.PULL)
socket.connect("tcp://localhost:5556")

#  Do 10 requests, waiting each time for a response
while True:
    resp = socket.recv()
    print(struct.unpack('iiiiii', resp))
