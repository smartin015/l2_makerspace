FROM l2ros:latest
#gcr.io/l2-making/base:latest

# python3 python3-pip
# gcc python3-dev musl-dev python3-setuptools

RUN pip3 install psycopg2-binary

COPY service.py /
CMD ["/bin/cd", "/volume/py_srvcli", "&&", ".", "install/setup.bash", "&&", "ros2", "run", "py_srvcli", "service"]
