FROM l2base:latest
#gcr.io/l2-making/base:latest

# python3 python3-pip
# gcc python3-dev musl-dev python3-setuptools

RUN pip3 install psycopg2-binary

CMD ["bash", "-c", "source /py_srvcli/install/setup.bash && source /l2_msgs/install/setup.bash && ros2 run py_srvcli service"]

COPY py_srvcli /py_srvcli
RUN cd /py_srvcli && colcon build --symlink-install
