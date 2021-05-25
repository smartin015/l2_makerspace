#include "app_hal.h"
#include "hw.h"
#include "log.h"
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <thread>

int cur_dir[NUM_J];
bool prev_step_pin[NUM_J];
int step_offs[NUM_J];


// For some reason, Ctrl+C isn't detected when
// running the native program via `pio` in Docker,
// so we have to handle it custom here.
void signal_callback_handler(int signum) {
   printf("Stop request detected, exiting...\n");
   exit(signum);
}

namespace hal {

void init() {
  // Nothing to do here; no hardware to initialize
}

void stepDir(int i, bool dir) {
  cur_dir[i] = (dir) ? 1 : -1;
}
void stepDn(int i) {
  if (prev_step_pin[i]) {
    hw::move_steps(i, cur_dir[i]);
  }
  prev_step_pin[i] = false;
}
void stepUp(int i) {
  prev_step_pin[i] = true;
}
void stepEnabled(int i, bool en) {}

bool readLimit(int i) {
  hw::sync();
  return hw::get_cur_cal(i);
}

int readEnc(int idx) {
  return hw::get_steps(idx); // TODO + step_offs[idx];
}

void writeEnc(int idx, int value) {
  step_offs[idx] = value - hw::get_steps(idx);
  LOG_DEBUG("Wrote %d to encoder %d", value, idx);
}

void disableInterrupts() {}
void enableInterrupts() {}


void runSteps(int hz, void(*cb)()) {
  int micro_pd = 1000000 / hz;
  while (true) {
    usleep(micro_pd); // NOTE: not actually a guarantee, likely to be consistently longer
    cb();
  }
}

std::thread t;
void startMainTimer(int hz, void(*cb)()) {
  t = std::thread(runSteps, hz, cb);
};

} // namespace hal


void setup();
void loop();
int main() {
  hw::init();
  signal(SIGINT, signal_callback_handler);
  signal(SIGTERM, signal_callback_handler);
  for (int i = 0; i < NUM_J; i++) {
    cur_dir[i] = 1;
    step_offs[i] = 0;
    prev_step_pin[i] = true;
  }
  setup();
  while (1) {
    hw::loop(); // Hidden hardware emulation
    loop();
  }
}
