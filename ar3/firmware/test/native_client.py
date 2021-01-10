import zmq
import time

context = zmq.Context()

#  Socket to talk to server
print("Connecting...")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

#  Do 10 requests, waiting each time for a response
while True:
    cmd = input('>')
    socket.send_string(cmd)

    resp = socket.recv()
    print(resp)
