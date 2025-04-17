SET(SOURCES screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_risk_165_142_png.c
    images/ui_img_signal_x16_png.c
    images/ui_img_wifi_x16_png.c
    images/ui_img_1678184450.c
    images/ui_img_thermometer_x32_png.c
    images/ui_img_humidity_x32_png.c
    images/ui_img_gauge_x16_png.c)

add_library(ui ${SOURCES})
