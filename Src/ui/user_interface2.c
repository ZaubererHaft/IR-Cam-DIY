#include <stdlib.h>
#include <sys/stat.h>

#include "common.h"
#include "user_interface.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "stdbool.h"
#include "navigation.h"
#include "battery_state.h"


typedef struct AnalogData_ {
    uint16_t stick_x;
    uint16_t stick_y;
    uint16_t battery_level;
} AnalogData;

static AnalogData analog_data;
static BatteryState battery_state;

float tMin = 15.0f;
float tMax = 40.0f;

static bool stick_z_pressed = false;
static uint32_t stick_lock_time;

static uint32_t frames = 0;
static uint32_t frames_counter = 0;

bool UserInterface_Init(void) {
    ILI9341_Init();
    ILI9341_Fill_Screen(BLACK);
    ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);

    Navigation_InitById(FRAME_DEFAULT);

    return true;
}

bool UserInterface_ShowingAnyMenuOrDialog(void) {
    return Navigation_CurrentFrame()->id != FRAME_DEFAULT;
}

void UserInterface_Loop(void) {
    // Read battery management chip
    const GPIO_PinState stat1 = !HAL_GPIO_ReadPin(BATTCHARGING_GPIO_Port, BATTCHARGING_Pin);
    const GPIO_PinState stat2 = !HAL_GPIO_ReadPin(BATTFULL_GPIO_Port, BATTFULL_Pin);
    const GPIO_PinState npg = !HAL_GPIO_ReadPin(EXT_POWER_GPIO_Port, EXT_POWER_Pin);

    BatteryState new_state;
    if (stat1 && !stat2 && npg) {
        new_state.chip_state = BATTERY_CHARGING;
    } else if (stat1 && !stat2 && !npg) {
        new_state.chip_state = BATTERY_LOW_BATTERY;
    } else if (!stat1 && !stat2 && npg) {
        new_state.chip_state = BATTERY_NOT_AVAILABLE;
    } else if (!stat1 && stat2 && npg) {
        new_state.chip_state = BATTERY_CHARGE_COMPLETE;
    } else {
        new_state.chip_state = BATTERY_DEFAULT;
    }

    new_state.battery_level = analog_data.battery_level;

    if (abs(new_state.battery_level - battery_state.battery_level) >= 50 || new_state.chip_state != battery_state.
        chip_state) {
        Navigation_BatteryStateChanged(new_state);
        battery_state = new_state;
    }

    // Read Navigation stick
    if (HAL_GetTick() - stick_lock_time > 100) {
        if (analog_data.stick_x - 2048 > 1500) {
            Navigation_StickLeftMoved();
        } else if (analog_data.stick_x - 2048 < -1500) {
            Navigation_StickRightMoved();
        }
        if (analog_data.stick_y - 2048 > 1500) {
            Navigation_StickDownMoved();
        } else if (analog_data.stick_y - 2048 < -1500) {
            Navigation_StickUpMoved();
        }

        if (stick_z_pressed) {
            Navigation_StickPressed();
            stick_z_pressed = false;
        }

        stick_lock_time = HAL_GetTick();
    }

    // animation loop
    Navigation_AnimationCallback();

    // FPS
    frames++;

    if (HAL_GetTick() - frames_counter > 1000) {
        Navigation_FPSChanged(frames);
        frames = 0;
        frames_counter = HAL_GetTick();
    }
}

void UserInterface_RedrawIRImageIfNecessary(float *image) {
}

void UserInterface_PutAnalogData(const uint16_t data[3]) {
    analog_data.stick_x = data[0];
    analog_data.stick_y = data[1];
    analog_data.battery_level = data[2];
}

void UserInterface_ConfigObserver(Config config) {
    Navigation_ConfigChanged(config);
}

void UserInterface_ShowError(const char *text) {
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == STICK_Z_Pin) {
        stick_z_pressed = true;
    }
}
