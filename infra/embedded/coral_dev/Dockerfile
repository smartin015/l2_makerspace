FROM ubuntu:18.04

RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg ca-certificates && \
   echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | tee /etc/apt/sources.list.d/coral-edgetpu.list && \
   curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add - && apt-get update && apt-get install -y --no-install-recommends libedgetpu1-std python3-pycoral git && rm -rf /var/lib/apt/lists/* 

RUN mkdir /coral && cd /coral && git clone https://github.com/google-coral/pycoral.git


