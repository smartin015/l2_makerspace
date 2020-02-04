FROM l2base:latest

RUN pip3 install todoist-python

ENV L2NAME=l2_tasks

CMD ["bash", "-c", "source /node/install/setup.bash && source /l2_msgs/install/setup.bash && ros2 run ${L2NAME} main"]

COPY . /node

RUN /templates/setup_boilerplate.sh node.py
