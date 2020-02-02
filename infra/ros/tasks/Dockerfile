FROM l2base:latest

RUN pip3 install todoist-python

ENV L2NAME=l2_example

CMD ["bash", "-c", "source /l2_tasks/install/setup.bash && source /l2_msgs/install/setup.bash && ros2 run ${L2NAME} main"]

COPY . /node

RUN /templates/setup_boilerplate.sh
