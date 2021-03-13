
Dev environment:
```
# Open up display for showing from container
xhost +

# Start container for webots simulator
 docker run --gpus=all --name l2ar3 -it --rm -e DISPLAY -v $(pwd)/webots:/usr/local/webots/resources/projects/ -v $(pwd)/config:/root/.config/Cyberbotics/ -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $(pwd):/volume l2ar3sim /bin/bash

# Start webots with default environment
webots
```

Separate process for running the accompanying nodes:
```
# Exec into existing webots container to run controller separately
docker exec -it l2ar3 /bin/bash

# (optional) Link an editable version of node.py into the installed location in the container
rm /node/install/l2_ar3/lib/python3.8/site-packages/l2_ar3/node.py
ln -s /volume/node.py /node/install/l2_ar3/lib/python3.8/site-packages/l2_ar3/node.py

# Run the l2_ar3 node program (webots controller) to accept commands and output telemetry
ros2 run l2_ar3 node
```

To launch URDF viewer for claw:
```
ros2 launch /volume/config/claw_rviz.launch.py
```
