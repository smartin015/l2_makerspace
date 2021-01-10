#include "app_hal.h"
#include "config.h"
#include "log.h"

# define OUTPUT 0
# define INPUT 1
# define HIGH true
#define LOW false

int cur_dir[NUM_J];
int steps[NUM_J];
bool prev_step_pin[NUM_J];

void digitalWrite(int pin, bool high) {
  for (int i = 0; i < NUM_J; i++) {
    if (pin == DIR_PIN[i]) {
      cur_dir[i] = (high) ? 1 : -1;
      return;
    } else if (pin == STEP_PIN[i]) {
      if (prev_step_pin[i] && !high) {
        steps[i] += cur_dir[i];
      }
      prev_step_pin[i] = high;
      return;
    }
  }
  LOG_ERROR("Wrote to unsimulated pin %d", pin);
}

void initHAL() {
  for (int i = 0; i < NUM_J; i++) {
    cur_dir[i] = 1;
    steps[i] = 0;
    prev_step_pin[i] = true;
  }
}

void setup();
void loop();
int main() {
  setup();
  while (1) {
    loop();
  }
}
