#include "app_hal.h"
#include "config.h"
#include "hw.h"
#include "log.h"

# define OUTPUT 0
# define INPUT 1
# define HIGH true
#define LOW false

int cur_dir[NUM_J];
bool prev_step_pin[NUM_J];
int step_offs[NUM_J];

void digitalWrite(int pin, bool high) {
  for (int i = 0; i < NUM_J; i++) {
    if (pin == DIR_PIN[i]) {
      cur_dir[i] = (high) ? 1 : -1;
      return;
    } else if (pin == STEP_PIN[i]) {
      if (prev_step_pin[i] && !high) {
        hw::move_target(i, cur_dir[i]);
      }
      prev_step_pin[i] = high;
      return;
    }
  }
  LOG_ERROR("Failed write to unknown pin %d (%d)", pin, high);
}

bool digitalRead(int pin) {
  hw::sync();
  for (int i = 0; i < NUM_J; i++) {
    if (pin == CAL_PIN[i]) {
      return hw::get_cur_cal(i);
    }
  }
  LOG_ERROR("Failed read of unknown pin %d", pin);
  return false;
}

int readEnc(int idx) {
  return hw::get_steps(idx) + step_offs[idx];
}

void writeEnc(int idx, int value) {
  step_offs[idx] = value - hw::get_steps(idx);
  LOG_DEBUG("Wrote %d to encoder %d", value, idx);
}

void setup();
void loop();
int main() {
  hw::init();
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
