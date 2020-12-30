#include "lvgl.h"
#include "app_hal.h"
#include "gui.h"

void setup() {
	lv_init();
	hal_setup();
}

void loop() {
	gui_create();
	hal_loop();
}

