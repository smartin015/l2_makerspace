from l2base:latest

# RUN bash -c "source /opt/ros/dashing/setup.bash && apt-get update && apt-get install -y ros-dashing-cv-bridge ros-dashing-librealsense2 ros-dashing-message-filters ros-dashing-image-transport libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev ros-dashing-realsense-camera-msgs ros-dashing-realsense-ros2-camera && rm -rf /var/lib/apt/lists/*"

ENV L2NAME=l2_depth_camera
COPY . /node
RUN /templates/setup_boilerplate.sh node.py generate.py \
  && mv /node/rvl.py /node/${L2NAME}/
