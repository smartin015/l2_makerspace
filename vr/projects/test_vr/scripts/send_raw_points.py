# 25 = PoolVector3Array type
# 4 4 Integer Array Length
# 8..8+length*12  4 Float X Coordinate
# 8..12+length*12 4 Float Y Coordinate
# 8..16+length*12 4 Float Z Coordinate
import math
import socket
import struct
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 4242

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP

fmt = "ii%df"

def gen_and_send(ts):
    data = []
    w = 64
    nvecs = w*w
    for i in range(nvecs):
        data += [
                (i % w) * 0.01, 
                (i // w) * 0.01, 
                math.sin(i/16.0 + ts/3.0) + math.cos(i/16.0 + ts/3.0) * 1.0
                ]
    packed = struct.pack(fmt % len(data), 25, nvecs, *data)
    sock.sendto(packed, (UDP_IP, UDP_PORT))

if __name__ == "__main__":
    while True:
        gen_and_send(time.time())
        time.sleep(0.1)

