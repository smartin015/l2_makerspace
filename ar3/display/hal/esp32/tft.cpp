#include <Arduino.h>
#include "tft.h"
#include <SPI.h>
#include <TFT_eSPI.h>

#define BUFSZ LV_HOR_RES_MAX * LV_VER_RES_MAX / 10

static lv_disp_drv_t disp_drv;
TFT_eSPI tft(LV_VER_RES_MAX, LV_HOR_RES_MAX);

static void tft_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
  uint16_t w = (area->x2 - area->x1 + 1);
  uint16_t h = (area->y2 - area->y1 + 1);
  tft.startWrite(); 
	tft.setAddrWindow(area->x1, area->y1, w, h);
	tft.pushColors(&color_p->full, w*h, true);
	tft.endWrite();
  lv_disp_flush_ready(drv);
}

void tft_init(void) {
  static lv_color_t disp_buf1[BUFSZ];
  static lv_disp_buf_t buf;
  lv_disp_buf_init(&buf, disp_buf1, NULL, BUFSZ);
  lv_disp_drv_init(&disp_drv);

  tft.init();
  tft.setRotation(1); // landscape
  disp_drv.buffer = &buf;
  disp_drv.flush_cb = tft_flush;
  lv_disp_drv_register(&disp_drv);
}

