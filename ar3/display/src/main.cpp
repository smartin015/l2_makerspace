#include "lvgl.h"
#include "app_hal.h"
#include "gui.h"
#include "ros.h"

void setup() {
	lv_init();
	hal_setup();
  ros_setup("", "");
	gui_create();
}

void loop() {
	hal_loop();
  ros_loop();
}

