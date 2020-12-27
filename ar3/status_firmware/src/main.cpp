#include "lv_conf.h"
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif 
#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define NUM_J 6

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * LV_VER_RES_MAX / 10];                     /*Declare a buffer for 1/10 screen size*/
lv_disp_drv_t disp_drv;
lv_obj_t *screenMain;
lv_obj_t *label;
lv_obj_t *gauge;
lv_obj_t *target_gauge;
lv_obj_t *limit_leds[NUM_J];
TFT_eSPI tft(LV_VER_RES_MAX, LV_HOR_RES_MAX);

#if USE_LV_LOG != 0
void print(lv_log_level_t level, const char * file, uint32_t line, const char * dsc){
  Serial.printf("%s@%d->%s\r\n", file, line, dsc);
  Serial.flush();
}
#endif

// https://daumemo.com/how-to-use-lvgl-library-on-arduino-with-an-esp-32-and-spi-lcd/
void flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p) {
    uint16_t w = (area->x2 - area->x1 + 1);
    uint16_t h = (area->y2 - area->y1 + 1);
		tft.startWrite(); 
		tft.setAddrWindow(area->x1, area->y1, w, h);
		tft.pushColors(&color_p->full, w*h, true);
		tft.endWrite();
    lv_disp_flush_ready(disp);
}

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1); // Landscape
  tft.fillScreen(TFT_BLACK);

	
  lv_init();
#if USE_LV_LOG != 0
  lv_log_register_print_cb(print); /* register print function for debugging */
#endif

  static lv_style_t style1;
  lv_style_init(&style1);
  lv_style_set_text_color(&style1, LV_STATE_DEFAULT, LV_COLOR_GRAY);
  lv_style_set_bg_color(&style1, LV_STATE_DEFAULT, LV_COLOR_BLACK);
  lv_style_set_border_color(&style1, LV_STATE_DEFAULT, lv_color_hex(0x101010));
  lv_style_set_pad_top(&style1, LV_STATE_DEFAULT, 1);
  lv_style_set_pad_bottom(&style1, LV_STATE_DEFAULT, 1);
  lv_style_set_pad_left(&style1, LV_STATE_DEFAULT, 1);
  lv_style_set_pad_right(&style1, LV_STATE_DEFAULT, 1);


  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX / 10);    /*Initialize the display buffer*/
  lv_disp_drv_init(&disp_drv);
  disp_drv.flush_cb = flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

	screenMain = lv_obj_create(NULL, NULL);
	lv_obj_add_style(screenMain, LV_OBJ_PART_MAIN, &style1);

  label = lv_label_create(screenMain, NULL);
  lv_obj_add_style(label, LV_LABEL_PART_MAIN, &style1);
  lv_label_set_long_mode(label, LV_LABEL_LONG_BREAK);
  lv_label_set_text(label, "ENOCONT"); // TODO sense
  lv_label_set_align(label, LV_LABEL_ALIGN_CENTER);
  lv_obj_set_size(label, 240, 40);
  lv_obj_set_pos(label, 0, 0);

	static lv_obj_t* limits = lv_cont_create(screenMain, NULL);
  lv_obj_add_style(limits, LV_CONT_PART_MAIN, &style1);
  lv_obj_set_auto_realign(limits, true);
  lv_cont_set_fit(limits, LV_FIT_TIGHT);
  lv_cont_set_layout(limits, LV_LAYOUT_COLUMN_MID);
  lv_obj_align(limits, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);

  static lv_color_t colors[] = {
    lv_color_hex(0x31005c),
    lv_color_hex(0x0045af),
    lv_color_hex(0x007bde),
    lv_color_hex(0x00acd3),
    lv_color_hex(0x00da91),
    lv_color_hex(0x22ff00),
  };
  int led_side = LV_VER_RES_MAX / (NUM_J + 5);
	for (int i = 0; i < NUM_J; i++) {
		limit_leds[i] = lv_led_create(limits, NULL);
    lv_obj_set_size(limit_leds[i], led_side+10, led_side);
    lv_obj_set_style_local_bg_color(limit_leds[i], LV_LED_PART_MAIN, LV_STATE_DEFAULT, colors[i]);
    lv_obj_set_style_local_shadow_color(limit_leds[i], LV_LED_PART_MAIN, LV_STATE_DEFAULT, colors[i]);
	}
  
  static lv_style_t gauge_style;
  lv_style_copy(&gauge_style, &style1);
  lv_style_set_line_width(&gauge_style, LV_STATE_DEFAULT, 2);
  gauge = lv_gauge_create(screenMain, NULL);
  lv_obj_add_style(gauge, LV_OBJ_PART_MAIN, &gauge_style);
  lv_obj_add_style(gauge, LV_GAUGE_PART_NEEDLE, &gauge_style);
  lv_gauge_set_needle_count(gauge, NUM_J, colors);
  lv_gauge_set_scale(gauge, 180, 9, 0); // 9 ticks, no labels
  int gaugesz = LV_VER_RES_MAX+55;
	lv_obj_set_size(gauge, gaugesz, gaugesz);
  lv_gauge_set_range(gauge, 0, 360);
  lv_gauge_set_angle_offset(gauge, 90);
  lv_gauge_set_critical_value(gauge, 361); // Highlight 360
  lv_obj_align(gauge, NULL, LV_ALIGN_IN_LEFT_MID, -gaugesz/2, 0);

	// TODO inlaid targets in gauge
	target_gauge = lv_gauge_create(screenMain, gauge);
  int tgaugesz = gaugesz / 2 + 10;
  lv_gauge_set_scale(target_gauge, 180, 5, 0); // 5 ticks, no labels
  lv_obj_set_size(target_gauge, tgaugesz, tgaugesz);
	lv_gauge_set_angle_offset(target_gauge, 90);
	lv_obj_align(target_gauge, NULL, LV_ALIGN_IN_LEFT_MID, -tgaugesz/2, 0);

  lv_scr_load(screenMain);
}

uint64_t last_update_targets = 0;
void loop() {
  lv_tick_inc(1);
  lv_task_handler();
  delay(1);
  bool update_targets = (millis() > last_update_targets + 1000);
	for (int i = 0; i < NUM_J; i++) {
		//lv_linemeter_set_value(limit_leds[i], ((now / 10) % 360) - 180); 
    uint64_t r = (millis() / 10 + 250 * i) % 720;
    if (r > 360) {
      r = 720 - r;
      lv_led_on(limit_leds[i]);
		} else {
			lv_led_off(limit_leds[i]);
		}
    lv_gauge_set_value(gauge, i, r);
    if (update_targets) {
		  lv_gauge_set_value(target_gauge, i, r);
      last_update_targets = millis();
    }
	}
}
