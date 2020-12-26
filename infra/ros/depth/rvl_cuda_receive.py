from numba import cuda
import rvl_cuda
from math import sqrt, ceil
import numpy as np
import cv2
import pyrealsense2 as rs
import socket
from datetime import datetime

DEPROJECT = False
SECTOR_BATCH = 3584
SECTOR_LEN = 64
dim = (480, 848)
UDP = ("0.0.0.0", 19823)
BUFSZ = SECTOR_LEN*8 # Recieve buffer size, accounting for byte->uint64
NUM_SECTOR = 3584
KB = (8, 16)
rvl_cuda.configure(KB, NUM_SECTOR)

def serve_forever(sock):
    encoded = {}
    decoded = {}
    i = 0
    print("UDP server listening")
    while(True):
        (data, src) = sock.recvfrom(BUFSZ)
        src=src[0]
        if len(data) != 512:
            continue

        if encoded.get(src) is None:
            print("New client", src)
            encoded[src] = np.zeros((SECTOR_BATCH, SECTOR_LEN), dtype=np.uint64)
            decoded[src] = np.zeros(dim, dtype=np.uint16)

        # TODO handle calibration packets (intrinsics / extrinsics)
        encoded[src][i,:] = np.frombuffer(data, dtype=np.uint64)
        i += 1
        if i >= SECTOR_BATCH:
            rvl_cuda.decode[4, int(SECTOR_BATCH/4)](encoded[src], decoded[src], DEPROJECT)
            cv2.imshow(src, decoded[src] / max(1, np.max(decoded[src])))
            cv2.waitKey(1)
            i = 0

if __name__ == "__main__":
    sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    sock.bind(UDP)
    try: 
        serve_forever(sock)
    finally: 
        cuda.close()


