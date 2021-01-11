#include "hw.h"
#include "log.h"
#include <string>
#include <iostream>
#include <unistd.h>
#include <zmq.hpp>

namespace hw {

bool cur_cal[NUM_J];
int steps[NUM_J];
int target[NUM_J];

bool get_cur_cal(int idx) { return cur_cal[idx]; }
bool get_steps(int idx) { return steps[idx]; }
bool move_target(int idx, int delta) { target[idx] += delta; }

#define PUSH_ADDR "tcp://*:5556"
#define PULL_ADDR "tcp://*:5557"

zmq::context_t context (1);
zmq::socket_t push_socket (context, ZMQ_PUSH);
zmq::socket_t pull_socket (context, ZMQ_PULL);

void init() {
  push_socket.bind(PUSH_ADDR);  
  pull_socket.bind(PULL_ADDR);
  LOG_DEBUG("HW telemetry initialized: push %s, pull %s", PUSH_ADDR, PULL_ADDR); 
  for (int i = 0; i < NUM_J; i++) {
    steps[i] = 0;
    target[i] = 0;
    cur_cal[i] = true;
  }
}

void loop() {
  zmq::message_t msg(NUM_J * sizeof(int));
  for (int i = 0; i < NUM_J; i++) {
    ((int*)msg.data())[i] = steps[i];
  }
  push_socket.send(msg, ZMQ_DONTWAIT);
  sync();
}

void sync() {
  zmq::message_t resp;
  if (pull_socket.recv(&resp, ZMQ_DONTWAIT)) {
    for (int i = 0; i < NUM_J; i++) {
      cur_cal[i] = (bool) ((char*)resp.data())[i];
    }
  } 
}

} // namespace hw
