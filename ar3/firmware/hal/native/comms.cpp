#include "comms.h"
#include "log.h"
#include <string>
#include <iostream>
#include <unistd.h>
#include <zmq.hpp>

#define ZMQ_ADDR "tcp://0.0.0.0:5555"

zmq::context_t context (1);
zmq::socket_t socket (context, ZMQ_REP);

namespace comms {

void init() {
  socket.bind(ZMQ_ADDR);  
  LOG_DEBUG("Controller ZMQ REP socket initialized: %s", ZMQ_ADDR); 
}

int read(uint8_t* buf, int buflen) {
  zmq::message_t request;
  if (socket.recv(&request, ZMQ_DONTWAIT)) {
    memcpy(buf, request.data(), request.size());
    //for (int i = 0; i < request.size(); i++) {
    //  printf("%02x", buf[i]);
    //}
    //printf("\n");
    return request.size();
  } else {
    return 0;
  }
}

void write(uint8_t* buf, int buflen) {
  zmq::message_t reply(buflen);
  memcpy(reply.data(), buf, buflen);
  socket.send(reply);
}

} // namespace comms
