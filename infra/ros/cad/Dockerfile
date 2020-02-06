FROM l2base:latest

RUN apt-get update && apt-get install unzip && rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/FreeCAD/FreeCAD/archive/master.zip && unzip master.zip

RUN cd FreeCAD-master && alias python=python3 && cmake . && make -j$(nproc --ignore=2)

ENV L2NAME=l2_example

CMD ["bash", "-c", "source /node/install/setup.bash && source /node/install/setup.bash && ros2 run ${L2NAME} main"]

COPY . /node

RUN /templates/setup_boilerplate.sh
