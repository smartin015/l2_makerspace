Realsense sender requires deps:

```
# Follow https://github.com/IntelRealSense/librealsense/issues/6964#issuecomment-707501049 to install python3 librealsense2 on nvidia jetson nano

# D435 depth resolution 848x480 is recommended (1280x720 for D415)
# For post processing tips see https://dev.intelrealsense.com/docs/depth-post-processing
# Should add edge-preserving filter (using disparity domain filtering) and skip holes in time

# numba may require installing conda (see also conda4aarch64 conda installer)
# https://numba.pydata.org/numba-doc/latest/user/installing.html#installing-on-linux-armv8-aarch64-platforms
# Conda must be python3.6 for builtin cv2 to work (`conda install python=3.6`)

pip3 install apriltag
```
