#ifndef CAM_UI_STATE_H
#define CAM_UI_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "main.h"


typedef enum BatteryChipState_ {
    BATTERY_CHARGING,
    BATTERY_LOW_BATTERY,
    BATTERY_NOT_AVAILABLE,
    BATTERY_DEFAULT
} BatteryChipState;

typedef struct BatteryState_ {
    BatteryChipState chip_state;
    uint16_t battery_level;
} BatteryState;

typedef struct UiState_ {
    Config app_config;
    BatteryState battery_state;
    uint32_t fps;
} UiState;

#endif //CAM_UI_STATE_H
