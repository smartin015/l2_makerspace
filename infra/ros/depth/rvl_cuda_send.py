from numba import cuda
import rvl_cuda
from math import sqrt, ceil
import numpy as np
import cv2
import pyrealsense2 as rs
import socket
from datetime import datetime

DEPROJECT = False
SECTOR_BATCH = 256
dim = (480, 848)
UDP = ("0.0.0.0", 19823)
BLOCKS_PER_GRID = (8, 7)
THREADS_PER_BLOCK = (8, 8)
NUM_SECTOR = 3584
KB = (8, 16)
rvl_cuda.configure(KB, NUM_SECTOR, {
    "ppx": 422.946, "ppy": 238.06,
    "fx": 426.192, "fy": 426.192,
})

def send_loop(sock):
    original = np.zeros(dim, dtype=np.uint16)
    encoded = np.zeros((NUM_SECTOR, rvl_cuda.SECTOR_LEN), dtype=np.uint64)
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, './a.bag')
    pipeline.start(config)
    while(True):
        try:
            has, frame = pipeline.try_wait_for_frames()
            if not has:
                break
            print(frame)
            data = np.asanyarray(frame.get_depth_frame().get_data())
            rvl_cuda.encode[BLOCKS_PER_GRID, THREADS_PER_BLOCK](data, encoded)
            for i in range(encoded.shape[0]):
                sock.sendto(encoded[i,:].tobytes(), UDP)
        except KeyboardInterrupt:
            return

if __name__ == "__main__":
    #import argparse
    #parser = argparse.ArgumentParser()
    #parser.add_argument("res")
    #args = parser.parse_args()
    sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    try: 
        send_loop(sock)
    finally: 
        cuda.close()


