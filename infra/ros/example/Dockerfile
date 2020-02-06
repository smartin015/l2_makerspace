FROM l2base:latest

ENV L2NAME=l2_example

CMD ["bash", "-c", "source /node/install/setup.bash && source /node/install/setup.bash && ros2 run ${L2NAME} node"]

COPY . /node

RUN /templates/setup_boilerplate.sh node.py
