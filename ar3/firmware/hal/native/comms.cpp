#include "comms.h"
#include "log.h"
#include <string>
#include <iostream>
#include <unistd.h>
#include <zmq.hpp>

#define ZMQ_ADDR "tcp://*:5555"

zmq::context_t context (1);
zmq::socket_t socket (context, ZMQ_REP);

void initComms() {
  socket.bind(ZMQ_ADDR);  
  LOG_DEBUG("Comms initialized: %s", ZMQ_ADDR); 
}

bool tryFetchCommand(char* buf, int buflen) {
  zmq::message_t request;
  if (socket.recv(&request, ZMQ_DONTWAIT)) {
    strncpy(buf, (char*)request.data(), buflen);
    return true;
  } else {
    return false;
  }
}

void sendResponse(char* buf, int buflen) {
  zmq::message_t reply(buflen);
  memcpy(reply.data(), buf, buflen);
  socket.send(reply);
}
