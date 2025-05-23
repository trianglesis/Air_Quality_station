// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 9.2.2
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
extern lv_obj_t * ui_Screen1;
extern lv_obj_t * ui_ArcCO2;
extern lv_obj_t * ui_LabelCo2Count;
extern lv_obj_t * ui_LabelCo2;
extern lv_obj_t * ui_LabelCo2Ppm;
extern lv_obj_t * ui_LabelSdFree;
extern lv_obj_t * ui_LabelLfsUsed;
extern lv_obj_t * ui_ImageAPMode;
extern lv_obj_t * ui_LabelApUsers;
extern lv_obj_t * ui_ImageLocalWiFI;
extern lv_obj_t * ui_LabelipAdress;
extern lv_obj_t * ui_ImageNoWiFi;
extern lv_obj_t * ui_BarHumidity;
extern lv_obj_t * ui_LabelTemperature;
extern lv_obj_t * ui_LabelHumidity;
extern lv_obj_t * ui_LabelPressure;
extern lv_obj_t * ui_LabelAirQualityIndx;
extern lv_obj_t * ui_BarTemperature;
extern lv_obj_t * ui_Image1;
extern lv_obj_t * ui_Image2;
extern lv_obj_t * ui_Image3;
// CUSTOM VARIABLES

// EVENTS

extern lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS
LV_IMG_DECLARE(ui_img_risk_165_142_png);    // assets/risk_165_142.png
LV_IMG_DECLARE(ui_img_signal_x16_png);    // assets/signal_x16.png
LV_IMG_DECLARE(ui_img_wifi_x16_png);    // assets/wifi_x16.png
LV_IMG_DECLARE(ui_img_1678184450);    // assets/no-wifi_x16.png
LV_IMG_DECLARE(ui_img_wet_to_dry_h90_png);    // assets/Wet_to_dry_h90.png
LV_IMG_DECLARE(ui_img_hot_to_cold_h90_png);    // assets/Hot_to_Cold_h90.png
LV_IMG_DECLARE(ui_img_thermometer_x32_png);    // assets/thermometer_x32.png
LV_IMG_DECLARE(ui_img_humidity_x32_png);    // assets/humidity_x32.png
LV_IMG_DECLARE(ui_img_gauge_x16_png);    // assets/gauge_x16.png

// UI INIT
void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
