#include "hw.h"
#include "log.h"
#include <string>
#include <iostream>
#include <unistd.h>
#include <zmq.hpp>
#include <chrono>

namespace hw {

bool cur_cal[NUM_J];
int steps[NUM_J];

bool get_cur_cal(int idx) { return cur_cal[idx]; }
int get_steps(int idx) { return steps[idx]; }

bool move_steps(int idx, int delta) { 
  steps[idx] += delta; 
}

#define PUSH_ADDR "tcp://*:5556"
#define PULL_ADDR "tcp://*:5557"

zmq::context_t context (1);
zmq::socket_t push_socket (context, ZMQ_PUSH);
zmq::socket_t pull_socket (context, ZMQ_PULL);

#define PUB_PD_MILLIS 100
std::chrono::steady_clock::time_point pgm_start;

uint64_t millis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - pgm_start).count();
}

void init() {
  pgm_start = std::chrono::steady_clock::now();
  push_socket.bind(PUSH_ADDR);  
  pull_socket.bind(PULL_ADDR);
  LOG_DEBUG("HW: pushing steps on %s with period %dms, pull limits on %s", 
            PUSH_ADDR, PUB_PD_MILLIS, PULL_ADDR); 
  for (int i = 0; i < NUM_J; i++) {
    steps[i] = 0;
    cur_cal[i] = true;
  }
}

uint64_t next_publish = 0;
void loop() {
  if (millis() > next_publish) {
    zmq::message_t msg(NUM_J * sizeof(int));
    for (int i = 0; i < NUM_J; i++) {
      ((int*)msg.data())[i] = steps[i];
    }
    push_socket.send(msg, ZMQ_DONTWAIT);  
    next_publish += PUB_PD_MILLIS;
  }
  sync();
}

void sync() {
  zmq::message_t resp;
  if (pull_socket.recv(&resp, ZMQ_DONTWAIT)) {
    for (int i = 0; i < NUM_J; i++) {
      cur_cal[i] = (bool) ((char*)resp.data())[i];
    }
    LOG_INFO("New limit state received: %d %d %d %d %d %d", 
        cur_cal[0], cur_cal[1], cur_cal[2], 
        cur_cal[3], cur_cal[4], cur_cal[5]);
  } 
}

} // namespace hw
