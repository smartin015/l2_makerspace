from numba import cuda
from math import sqrt
import numpy as np
import cv2
import pyrealsense2 as rs

import jetson.inference
import jetson.utils

# Note that column order is height-first
PX_B = 2
input_dim_realsense_reg = (480, 848)
dim = input_dim_realsense_reg

from datetime import datetime

timings = []

pipeline = rs.pipeline()
config = rs.config()
#config.enable_stream(rs.stream.depth, dim[1], dim[0], rs.format.z16, 30)
config.enable_stream(rs.stream.color, dim[1], dim[0], rs.format.bgr8, 30)
pipeline.start(config)

# Output image for overlay (float4)
# https://www.linkedin.com/pulse/realtime-semantic-segmentation-jetson-nano-python-c-dustin-franklin

# https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
# face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
net = jetson.inference.segNet("fcn-resnet18-mhp-512x320")
        # detect room feateures: "fcn-resnet18-sun-512x400"
objmap = [net.GetClassDesc(i) for i in range(net.GetNumClasses())]
handClass = objmap.index("hand")
for i in range(net.GetNumClasses()):
    print("{}: {}".format(i, net.GetClassDesc(i)))
net.SetOverlayAlpha(150.0)

print("Running on img shape {} (Ctl+C to stop)...".format(dim))
while True:
    try:
        has, frame = pipeline.try_wait_for_frames()
        if not has:
            break
        start = datetime.now()
        # data = np.asanyarray(frame.get_depth_frame().get_data())
        col = frame.get_color_frame()
        col = np.asanyarray(col.get_data())
        
        # TODO Get direct in GPU for faster processing
        # https://github.com/IntelRealSense/librealsense/blob/f361e56f8a2fb5093a1d663293421c92dd432bba/examples/gl/readme.md
        col = jetson.utils.cudaFromNumpy(col)
        out = jetson.utils.cudaAllocMapped(width=col.shape[1], height=col.shape[0], format="gray8")


        #detections = net.Detect(col, overlay="box,labels,conf")
        net.Process(col)
        net.Mask(out)
        
        # Convert to 8-bit
        #data8 = (data/256).astype('uint8')
        #ret, thresh = cv2.threshold(data8,0, 128, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        # noise removal
        #kernel = np.ones((3,3),np.uint8)
        #opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
        # sure background area
        #sure_bg = cv2.dilate(opening,kernel,iterations=3)
        # Finding sure foreground area
        #dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        #ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
        # Finding unknown region
        #sure_fg = np.uint8(sure_fg)
        #unknown = cv2.subtract(sure_bg,sure_fg)
        # Marker labelling
        #ret, markers = cv2.connectedComponents(sure_fg)
        # Add one to all labels so that sure background is not 0, but 1
        #markers = markers+1
        # Now, mark the region of unknown with zero
        #markers[unknown==255] = 0
        #result = np.stack((data8,)*3, axis=-1) #np.zeros(list(data8.shape) + [3], dtype=np.uint8)
        #markers = cv2.watershed(result, markers)
        #gray = cv2.cvtColor(col, cv2.COLOR_RGB2GRAY)
        #found = face_cascade.detectMultiScale(gray, minSize=(20,30))
        #print(len(found))
        #for (x, y, width, height) in found:
        #    cv2.rectangle(col, (x, y), (x+height, y+width), (0, 255, 0), 5)
        res = jetson.utils.cudaToNumpy(out)

        # Highlight hands
        res[res == handClass] = 255

        cv2.putText(res, '{:.0f} FPS'.format(net.GetNetworkFPS()), (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow("result", res)#cv2.applyColorMap(dist_transform, cv2.COLORMAP_JET))
        


        cv2.waitKey(1)
        timings.append(datetime.now() - start)
    except KeyboardInterrupt:
        break

print("(Actual image shape received: {} dtype {})".format(data.shape, data.dtype))

# np.set_printoptions(threshold=np.inf)
print("result ({} initial, {} post-avg, {} samples)".format(
          timings[0], np.mean(timings[1:]), len(timings[1:])))

