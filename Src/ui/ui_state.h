#ifndef CAM_UI_STATE_H
#define CAM_UI_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "main.h"

typedef enum UI_FRAME_ {
    FRAME_DEFAULT,
    FRAME_MENU_OVERVIEW,
    FRAME_MENU_GALLERY
} UI_FRAME;

typedef struct AnalogData_ {
    uint16_t stick_x;
    uint16_t stick_y;
    uint16_t battery_level;
} AnalogData;

typedef struct BatteryState_ {
    GPIO_PinState old_stat1;
    GPIO_PinState old_stat2;
    GPIO_PinState old_npg;
    bool initial_draw_battery;
    uint16_t battery_level;
    uint16_t old_battery_leve;
    bool last_anim_charge;
} BatteryState;

typedef struct UiState_ {
    UI_FRAME current_frame;
    bool need_redraw;
    uint32_t fps;
    Config app_config;
    AnalogData analog_data;
    BatteryState battery_state;
} UiState;

#endif //CAM_UI_STATE_H
