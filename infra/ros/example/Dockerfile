FROM l2base:latest

CMD ["bash", "-c", "source /node/install/setup.bash && source /node/install/setup.bash && ros2 run l2_example node"]

COPY . /node
RUN cd /node && colcon build --symlink-install
