#include <Arduino.h>
#include "tft.h"

void hal_setup(void) {
  Serial.begin(115200);
  tft_init();
}

void hal_loop(void) {
  while(1) {
    delay(5);
    lv_tick_inc(5);
		lv_task_handler();
  }
}
