FROM l2base:latest

RUN pip3 install psycopg2-binary

CMD ["bash", "-c", "source /py_srvcli/install/setup.bash && source /l2_msgs/install/setup.bash && ros2 run py_srvcli service"]

ENV L2NAME=l2_storage
COPY . /node
RUN /templates/setup_boilerplate.sh
