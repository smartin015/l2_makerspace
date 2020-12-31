/**
 * @file disp.h
 * 
 */

#ifndef DISP_H
#define DISP_H

#include "lvgl.h"

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif 
#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

void tft_init(void);

#endif
