import zmq
import time
import struct
from threading import Thread

NUM_J = 6
PRINT_PD = 2
context = zmq.Context()

#  Socket to talk to server
print("Connecting...")
pull_socket = context.socket(zmq.PULL)
pull_socket.connect("tcp://localhost:5556")

push_socket = context.socket(zmq.PUSH)
push_socket.connect("tcp://localhost:5557")

#  Do 10 requests, waiting each time for a response
def recv_thread():
    c = 0
    targets = [0]*NUM_J
    nextPrint = time.time()
    while True:
        resp = pull_socket.recv()
        c += 1
        targets[:] = struct.unpack('i'*NUM_J, resp)
        if time.time() > nextPrint:
          print(targets, "\t", c/PRINT_PD, "hz")
          nextPrint += PRINT_PD
          c = 0
rt = Thread(target=recv_thread, daemon=True).start()

lims = [0]*NUM_J
while True:
    cmd = set(input())
    lims[:] = [str(i+1) in cmd for i in range(NUM_J)]
    print(lims)
    push_socket.send(struct.pack('b'*NUM_J, *lims))
    print("Updated limits:", lims)
