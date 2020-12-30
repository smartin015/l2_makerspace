#include "lvgl.h"
#include "app_hal.h"
#include "gui.h"
#include "ros_hal.h"

void setup() {
	lv_init();
	hal_setup();
  ros_hal::init();
	gui_create();
}

void loop() {
	hal_loop();
  ros_hal::spin();
}

