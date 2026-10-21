#include <sys/stat.h>

#include "common.h"
#include "user_interface.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "stdbool.h"
#include "gallery.h"
#include "menu.h"
#include "ui_state.h"

static UiState state;

float tMin = 15.0f;
float tMax = 40.0f;

bool UserInterface_Init(void) {
    ILI9341_Init();
    ILI9341_Fill_Screen(BLACK);
    ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);

    state.current_frame = FRAME_MENU_GALLERY;
    state.need_redraw = true;

    return true;
}

bool UserInterface_ShowingAnyMenuOrDialog(void) {
    return state.current_frame != FRAME_DEFAULT;
}


void UserInterface_Draw(void) {
    if (state.need_redraw) {
        Common_Draw(&state);

        if (state.current_frame == FRAME_MENU_OVERVIEW) {
            Menu_Draw(&state);
        } else if (state.current_frame == FRAME_MENU_GALLERY) {
            Gallery_Draw(&state);
        }

        state.need_redraw = false;
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
        if (state.current_frame == FRAME_DEFAULT) {
            state.current_frame = FRAME_MENU_OVERVIEW;
        }

        state.need_redraw = true;
    }
}
