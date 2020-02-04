FROM l2base:latest

RUN git clone https://github.com/FreeCAD/FreeCAD && cd FreeCAD && cmake . && make -j$(nproc --ignore=2)

ENV L2NAME=l2_example

CMD ["bash", "-c", "source /node/install/setup.bash && source /node/install/setup.bash && ros2 run ${L2NAME} main"]

COPY . /node

RUN /templates/setup_boilerplate.sh
