# 25 = PoolVector3Array type
# 4 4 Integer Array Length
# 8..8+length*12  4 Float X Coordinate
# 8..12+length*12 4 Float Y Coordinate
# 8..16+length*12 4 Float Z Coordinate
import math
import socket
import struct
import time
import sys
from os import path

sys.path.append(path.join(path.dirname(__file__), '../addons/rvl/'))
import rvl

UDP_IP = "35.245.27.12"
UDP_PORT = 4242
SCALE_FACTOR = 1000

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)

sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP

fmt = "ii%dB"

def gen_and_send(ts):
    data = []
    w = 128
    nvecs = w*w
    for i in range(w):
        for j in range(w):
            v = (math.sin(i/16.0 + ts/3.0) + math.cos(j/16.0 + ts/3.0)) * 0.1
            data += [int(v*SCALE_FACTOR)]
    rvl.Clear()
    rvl.plain = data
    rvl.CompressRVL()
    packed = struct.pack(fmt % len(rvl.encoded), 20, nvecs, *rvl.encoded)
    print(len(packed))
    sock.sendto(packed, (UDP_IP, UDP_PORT))

if __name__ == "__main__":
    while True:
        gen_and_send(time.time())
        time.sleep(0.1)

