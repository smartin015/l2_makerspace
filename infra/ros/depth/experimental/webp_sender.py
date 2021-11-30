import struct
import sys
import time

import socket
sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
GD_VARIANT = "ii%dB"
GD_RAW_ARRAY = 20
UDP_DEST = (sys.argv[1], 4242)
print(UDP_DEST)
with open('512512.webp', 'rb') as f:
    data = f.read()

print(type(data))
while True:
    try:
        encoded = bytes([1]) + data
        print(encoded[0])
        gdvar = struct.pack(GD_VARIANT % len(encoded), GD_RAW_ARRAY, len(encoded), *encoded)
        sock.sendto(gdvar, UDP_DEST)
        print(len(encoded))
        time.sleep(1.0)
    except socket.gaierror:
        pass

