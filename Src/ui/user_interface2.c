#include <stdlib.h>
#include <sys/stat.h>

#include "common.h"
#include "user_interface.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "stdbool.h"
#include "gallery.h"
#include "menu.h"
#include "navigation.h"
#include "ui_state.h"

static UiState state;

float tMin = 15.0f;
float tMax = 40.0f;

static bool stick_z_pressed = false;
static uint32_t stick_lock_time;

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

    // Read Navigation stick
    if (HAL_GetTick() - stick_lock_time > 100) {
        if (state.analog_data.stick_x - 2048 > 1500) {
            Navigation_StickLeftMoved();
        } else if (state.analog_data.stick_x - 2048 < -1500) {
            Navigation_StickRightMoved();
        }
        if (state.analog_data.stick_y - 2048 > 1500) {
            Navigation_StickDownMoved();
        } else if (state.analog_data.stick_y - 2048 < -1500) {
            Navigation_StickUpMoved();
        }

        if (stick_z_pressed) {
            Navigation_StickPressed();
            stick_z_pressed = false;
        }

        stick_lock_time = HAL_GetTick();
    }
}

void UserInterface_RedrawIRImageIfNecessary(float *image) {
}

void UserInterface_PutAnalogData(const uint16_t data[3]) {
    state.analog_data.stick_x = data[0];
    state.analog_data.stick_y = data[1];
    state.analog_data.battery_level = data[2];
}

void UserInterface_ConfigObserver(Config config) {
}

void UserInterface_ShowError(const char *text) {
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == STICK_Z_Pin) {
        stick_z_pressed = true;
    }
}
