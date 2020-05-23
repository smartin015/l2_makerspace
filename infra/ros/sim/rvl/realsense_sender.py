import pyrealsense2 as rs
import numpy as np
import rvl
import struct
import sys
import time

import socket
sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
GD_VARIANT = "ii%dB"
GD_RAW_ARRAY = 20
UDP_DEST = (sys.argv[1], 4242)

pipeline = rs.pipeline()
config = rs.config()
DIMS = (256,144)
STRIDE = 4
config.enable_stream(rs.stream.depth, DIMS[0], DIMS[1], rs.format.z16, 90)
profile = pipeline.start(config)

print("stream intrinsics: ", profile.get_streams()[0].as_video_stream_profile().get_intrinsics())

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("z16 depth Scale is: " , depth_scale)


rvl.Init(DIMS[0]/STRIDE,DIMS[1]/STRIDE, 0, 0)
i = 0
avgtime = 0

def rebin(a, stride):
    return a.reshape(a.shape[0] // stride, stride, a.shape[1] // stride, stride).max(3).max(1)

while True:
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    if not depth: 
        continue
    if len(rvl.plain) > 0:
        rvl.prev = rvl.plain
    # depth.get_data() returns array[height][width]
    # ravel() by default does row-major flattening (i.e. the last array index changes fastest)
    # So the flattened array should have the first image row, then second, then third etc.
    rvl.plain = rebin(np.asanyarray(depth.get_data()), STRIDE)

    if i == 10:
        from PIL import Image
        print("maxval", rvl.plain.max())
        Image.fromarray((rvl.plain * 255.0 / rvl.plain.max()).astype('uint8'), 'L').save("/tmp/testimg.png")
        first = False
        print("Image saved")

    rvl.plain = rvl.plain.ravel()
    i = i + 1
    start = time.time()
    rvl.Compress()
    end = time.time()
    avgtime = 0.9*avgtime + 0.1*int((end-start)*1000)
    try:
        gdvar = struct.pack(GD_VARIANT % len(rvl.encoded), GD_RAW_ARRAY, len(rvl.encoded), *rvl.encoded)
        if len(rvl.plain) > 1527:
            print(4*len(rvl.plain), "->", len(rvl.encoded), "(", int(avgtime), "ms) -> gd ", len(gdvar), "vs", rvl.plain[1527])
        sock.sendto(gdvar, UDP_DEST)
    except socket.gaierror:
        pass

