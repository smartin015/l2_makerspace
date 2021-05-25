#include "comms.h"
#include "log.h"
#include <string>
#include <iostream>
#include <unistd.h>
#include <zmq.hpp>

#define ZMQ_PULL_ADDR "tcp://0.0.0.0:5559"
#define ZMQ_PUSH_ADDR "tcp://0.0.0.0:5558"

zmq::context_t context (1);
zmq::socket_t push (context, ZMQ_PUSH);
zmq::socket_t pull (context, ZMQ_PULL);

namespace comms {

void init() {
  int linger = 0;
  LOG_DEBUG("Init comms PUSH socket: %s", ZMQ_PUSH_ADDR); 
  zmq_setsockopt(push, ZMQ_LINGER, &linger, sizeof(linger));
  push.bind(ZMQ_PUSH_ADDR);  
  LOG_DEBUG("Init comms PULL socket: %s", ZMQ_PULL_ADDR); 
  zmq_setsockopt(pull, ZMQ_LINGER, &linger, sizeof(linger));
  pull.bind(ZMQ_PULL_ADDR);
}

int read(uint8_t* buf, int buflen) {
  zmq::message_t request;
  if (pull.recv(request, zmq::recv_flags::dontwait)) {
    memcpy(buf, request.data(), request.size());
    return request.size();
  } else {
    return 0;
  }
}

void write(uint8_t* buf, int buflen) {
  zmq::message_t reply(buflen);
  memcpy(reply.data(), buf, buflen);
  push.send(reply, zmq::send_flags::dontwait);
}

} // namespace comms
