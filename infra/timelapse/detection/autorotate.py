import numpy as np
import cv2
import math
import apriltag
from scipy import ndimage

img_before = cv2.imread('sample.png')
img_gray = cv2.cvtColor(img_before, cv2.COLOR_BGR2GRAY)
img_edges = cv2.Canny(img_gray, 100, 100, apertureSize=3)
lines = cv2.HoughLinesP(img_edges, 1, math.pi / 180.0, 100, minLineLength=100, maxLineGap=5)



angles = []
for [[x1, y1, x2, y2]] in lines:
    cv2.line(img_before, (x1, y1), (x2, y2), (255, 0, 0), 3)
    angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    angles.append(angle)

median_angle = np.median(angles)
print(f"Angle is {median_angle:.04f}")
img_rotated = ndimage.rotate(img_before, median_angle)

tricheck = cv2.imread('bluetriangle.png')
tw, th = tricheck.shape[0:2]

res = cv2.matchTemplate(img_rotated,tricheck,cv2.TM_CCOEFF_NORMED)
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
top_left = max_loc
bottom_right = (top_left[0] + tw, top_left[1] + th)
cv2.rectangle(img_rotated,top_left, bottom_right, 255, 2)
print("Template is",min_val, max_val, min_loc, max_loc)


# Use the rough size of the sample image to inform the size of the display and
# crop to it
cropsz = (int(tw*6), int(th*1.8))
cropxy = (top_left[0], top_left[1]+int(1.2*th))
print("Cropping to", cropsz, "from", cropxy)

img_cropped = cv2.cvtColor(img_rotated[cropxy[1]:cropxy[1]+cropsz[1], cropxy[0]:cropxy[0]+cropsz[0]], cv2.COLOR_BGR2GRAY)

if max_val > 0.98:
  print("Match on upside-down blue triangle, rotating 180 degrees")
  img_cropped = ndimage.rotate(img_cropped, 180)

# Divide into 4 digits and do simple recognition on them
digit_w = int(img_cropped.shape[1] / 4)
print("Digit width:", digit_w, "of image size", img_cropped.shape)
digits = [None, None, None, None]
for i in range(4):
  print(i*digit_w, (i+1)*digit_w)
  digits[i] = img_cropped[:, i*digit_w:((i+1)*digit_w)]

  # TODO compute location histogram, e.g. avg color of 100 cubes of the image
  # Compare to inmemory cache - if new values, add it to the map and save a copy of the digit for later comparison

cv2.imwrite('digit3.jpg', digits[2])  
