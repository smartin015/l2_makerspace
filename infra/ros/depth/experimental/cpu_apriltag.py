import pyrealsense2 as rs
import numpy as np
import rvl
import struct
import sys
import time
import socket
import cv2
import apriltag
sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
GD_VARIANT = "ii%dB"
GD_RAW_ARRAY = 20
UDP_DEST = (sys.argv[1], 4242)

pipeline = rs.pipeline()
config = rs.config()

# Use `rs-enumerate-devices` console command to get available resolution @ framerate
DEPTH_DIMS = (1280, 720) # (256,144)
DEPTH_HZ = 30
COLOR_DIMS = (1280, 720)
COLOR_HZ = 30
STRIDE = 10 # Decimate the data by this amount before compressing
config.enable_stream(rs.stream.depth, DEPTH_DIMS[0], DEPTH_DIMS[1], rs.format.z16, DEPTH_HZ)
config.enable_stream(rs.stream.color, COLOR_DIMS[0], COLOR_DIMS[1], rs.format.bgr8, COLOR_HZ)
profile = pipeline.start(config)

intrinsics = profile.get_streams()[0].as_video_stream_profile().get_intrinsics()
intrinsics_apriltag = [intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]
print("Camera intrinsics:", intrinsics)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("z16 depth Scale is: " , depth_scale)


rvl.Init(DEPTH_DIMS[0]/STRIDE,DEPTH_DIMS[1]/STRIDE, 0, 0)
i = 0
avgtime = 0

detector = apriltag.Detector()

def rebin(a, stride):
    return a.reshape(a.shape[0] // stride, stride, a.shape[1] // stride, stride).max(3).max(1)

lastData = None
while True:
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    color = frames.get_color_frame()
    if not depth: 
        continue
    if len(rvl.plain) > 0:
        rvl.prev = rvl.plain
    # depth.get_data() returns array[height][width]
    # ravel() by default does row-major flattening (i.e. the last array index changes fastest)
    # So the flattened array should have the first image row, then second, then third etc.
    data = rebin(np.asanyarray(depth.get_data()), STRIDE)
    if lastData is not None:
        rvl.plain = (data - lastData).ravel()
    else:
        rvl.plain = data.ravel()

    if color:
        # Test - detect fiducial & get corner points
        colorData = np.asanyarray(color.get_data())
        detections = detector.detect(cv2.cvtColor(colorData, cv2.COLOR_BGR2GRAY))
        for d in detections:
            cv2.circle(colorData, tuple([int(x) for x in d.center]), 20, (255, 0, 0), 2)
            for cor in d.corners:
                cv2.circle(colorData, tuple([int(x) for x in cor]), 5, (255, 0, 0), -1)
            #print(d.tag_family, d.tag_id, d.center, d.corners)
            # Transformation matrix localized from the fiducial
            print(detector.detection_pose(d, intrinsics_apriltag))
        cv2.imshow('colorData', colorData)
        cv2.waitKey(1) # Required to show image

    # if i % 10 == 0:
    # print("maxval", rvl.plain.max())
    #(rvl.plain * 255.0 / rvl.plain.max()).astype('uint8'))
    # .save("/tmp/testimg.png")
    # print("Image saved")
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
    lastData = data
