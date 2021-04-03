FROM python:3

RUN apt-get update && apt-get -y install --no-install-recommends python3-pip && pip3 install websockets paho-mqtt pyzmq && rm -rf /var/lib/apt/lists*

