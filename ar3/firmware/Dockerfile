FROM l2base:latest

RUN apt-get update && apt-get install -y --no-install-recommends curl libzmq3-dev && rm -rf /var/lib/apt/lists/* \
  && python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
ENV PATH="${PATH}:~/.platformio/penv/bin"
