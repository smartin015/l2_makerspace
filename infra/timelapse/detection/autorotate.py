import numpy as np
import cv2
import math
import apriltag
from scipy import ndimage

detector = apriltag.Detector()
colorData = cv2.imread('apriltag_example.jpg')
gray = cv2.cvtColor(colorData, cv2.COLOR_BGR2GRAY)
cv2.imshow("Before", gray)  
key = cv2.waitKey(0)

"""
detections = detector.detect()

fx = 1394.6027293299926
fy = 1394.6027293299926
ppx = 995.588675691456
ppy = 599.3212928484164
intrinsics_apriltag = [fx, fy, ppx, ppy]


print(detector.detection_pose(d, intrinsics_apriltag))
"""


"""
img_gray = cv2.cvtColor(img_before, cv2.COLOR_BGR2GRAY)
img_edges = cv2.Canny(img_gray, 100, 100, apertureSize=3)
lines = cv2.HoughLinesP(img_edges, 1, math.pi / 180.0, 100, minLineLength=100, maxLineGap=5)

angles = []

for [[x1, y1, x2, y2]] in lines:
    cv2.line(img_before, (x1, y1), (x2, y2), (255, 0, 0), 3)
    angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    angles.append(angle)

cv2.imshow("Detected lines", img_before)    
key = cv2.waitKey(0)

median_angle = np.median(angles)
img_rotated = ndimage.rotate(img_before, median_angle)

print(f"Angle is {median_angle:.04f}")
cv2.imwrite('rotated.jpg', img_rotated)  
"""
