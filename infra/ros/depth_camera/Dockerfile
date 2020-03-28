ARG L2PKG=l2_depth_camera
ARG L2NODES=node.py generate.py
FROM l2base-pre
FROM l2base-post

# RUN bash -c "source /opt/ros/dashing/setup.bash && apt-get update && apt-get install -y ros-dashing-cv-bridge ros-dashing-librealsense2 ros-dashing-message-filters ros-dashing-image-transport libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev ros-dashing-realsense-camera-msgs ros-dashing-realsense-ros2-camera && rm -rf /var/lib/apt/lists/*"

COPY --from=0 /node/install node/install
# RUN /templates/setup_boilerplate.sh node.py generate.py \
#   && mv /node/rvl.py /node/${L2NAME}/
