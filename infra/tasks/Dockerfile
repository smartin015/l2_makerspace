FROM l2base:latest
#gcr.io/l2-making/base:latest

# python3 python3-pip
# gcc python3-dev musl-dev python3-setuptools

RUN pip3 install psycopg2-binary

CMD ["bash", "-c", "source /l2_tasks/install/setup.bash && source /l2_msgs/install/setup.bash && ros2 run l2_tasks service"]

COPY l2_tasks /l2_tasks
RUN cd /l2_tasks && colcon build --symlink-install
