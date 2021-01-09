#include "app_hal.h"
#include "log.h"

# define OUTPUT 0
# define INPUT 1
# define HIGH true
#define LOW false

void digitalWrite(int pin, bool high) {
  LOG_DEBUG("TODO digitalWrite");
}


void initHAL() {

}

void setup();
void loop();
int main() {
  setup();
  while (1) {
    loop();
  }
}
