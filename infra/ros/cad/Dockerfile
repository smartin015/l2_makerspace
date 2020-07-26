FROM l2base:latest

git clone https://github.com/FreeCAD/FreeCAD

RUN apt-get update && apt-get install -y --no-install-recommends freecad-python3 python3-pyqt5 && apt-cache clean

ENV L2NAME=l2_cad

CMD ["bash", "-c", "source /node/install/setup.bash && source /node/install/setup.bash && ros2 run ${L2NAME} main"]

COPY . /node

# RUN /templates/setup_boilerplate.sh
