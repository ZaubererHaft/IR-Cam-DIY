#ifndef CAM_UI_STATE_H
#define CAM_UI_STATE_H

#include <stdint.h>

typedef enum BatteryChipState_ {
    BATTERY_CHARGING,
    BATTERY_LOW_BATTERY,
    BATTERY_NOT_AVAILABLE,
    BATTERY_CHARGE_COMPLETE,
    BATTERY_DEFAULT
} BatteryChipState;

typedef struct BatteryState_ {
    BatteryChipState chip_state;
    uint16_t battery_level;
} BatteryState;


#endif //CAM_UI_STATE_H
