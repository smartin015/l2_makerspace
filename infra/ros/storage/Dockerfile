FROM l2base:latest

RUN pip3 install psycopg2-binary

ENV L2NAME=l2_storage

CMD ["bash", "-i", "-c", "source /node/install/setup.bash && ros2 run ${L2NAME} node"]

COPY . /node
RUN /templates/setup_boilerplate.sh node.py 
