#include "common.h"
#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "navigation.h"
#include "ui_state.h"

#define BATT_MAX_LVL 3583

static uint32_t last_batt_anim_time = 0;
static BatteryState battery_state;
static bool redraw_battery_state = true;

static void Common_CleanUp(void) {
    ILI9341_Draw_Rectangle(0, 0, 320, 20, BLACK);
    ILI9341_Draw_Rectangle(0, 220, 320, 20, BLACK);
    ILI9341_Draw_Rectangle(260, 0, 60, 240, BLACK);
}

static void Common_DrawHeatmap(UiState *state) {
    HeatmapFunction heatmap = Heatmap_GetByIndex(state->app_config.heatmap_index);

    int x = 270;
    int y = 50;

    char buff[20] = {};
    snprintf(buff, sizeof(buff), "%i", (int) tMin);
    ILI9341_Draw_Text(buff, x + 20, y, WHITE, 2, BLACK);

    const int chunks = 30;
    const float step = (tMax - tMin) / (float) chunks;
    for (int i = 0; i < chunks; ++i) {
        const float val = tMin + (float) i * step;
        ILI9341_Draw_Rectangle(x, y, 15, 4, heatmap(val));
        y += 4;
    }

    const float mid = (tMax + fabsf(tMin)) / 2.0f;
    snprintf(buff, sizeof(buff), "%i", (int) mid);
    ILI9341_Draw_Text(buff, x + 20, y - 65, WHITE, 2, BLACK);

    snprintf(buff, sizeof(buff), "%i", (int) tMax);
    ILI9341_Draw_Text(buff, x + 20, y - 16, WHITE, 2, BLACK);
}

static void Common_DrawFPS(UiState *state) {
    char buff[15] = "FPS: ";
    snprintf(&buff[5], 10, "%lu", state->fps);
    ILI9341_Draw_Text(buff, 260, 225, WHITE, 1, BLACK);
}

static void Common_DrawHeader(void) {
    ILI9341_Draw_Text(" D-CAM ", 0, 0, WHITE, 2, BLACK);
}

static void Common_DrawRecordState(UiState *state) {
    if (Navigation_CurrentFrame()->id == FRAME_DEFAULT) {
        ILI9341_Draw_Filled_Circle(304, 12, 8, RED);
    } else {
        ILI9341_Draw_Rectangle(300, 4, 4, 16, LIGHTGREY);
        ILI9341_Draw_Rectangle(306, 4, 4, 16, LIGHTGREY);
    }
}

static void Common_DrawBattery() {
    static bool anim_switch = false;
    uint32_t x = 260;
    uint32_t y = 3;

    // Clean up
    ILI9341_Draw_Rectangle(100, 0, 270, 25, BLACK);

    // Battery frame
    uint16_t battery_bar_color = WHITE;

    if (battery_state.chip_state == BATTERY_CHARGE_COMPLETE) {
        battery_bar_color = GREEN;
    }

    // ca. 4.2 V Ladeschlusspannung
    // spannungsteiler 100 - 220 -> max 2.88 V Spannung
    // -> 0.875 Maximal von 3.3 V erreichbar
    // -> 0.875 * 4096 = 3583 sind 100 % batteriespannung
    char buff[20] = {};

    // Battery level in numbers
    const uint32_t lvl_perc = (uint32_t) ((float) battery_state.battery_level / (float) BATT_MAX_LVL * 100.0f);
    snprintf(buff, sizeof(buff), "%lu%%", lvl_perc);
    ILI9341_Draw_Text(buff, x - 55, 0, WHITE, 2, BLACK);

    // Battery frame
    ILI9341_Draw_Hollow_Rectangle_Coord(x - 3, 0, x + 18, y + 14 + 2, WHITE);
    ILI9341_Draw_Rectangle(x + 18, y + 1, 3, 10, WHITE);

    if (battery_state.chip_state == BATTERY_CHARGING) {
        // Two bars
        ILI9341_Draw_Rectangle(x, y, 4, 14, battery_bar_color);
        ILI9341_Draw_Rectangle(x + 6, y, 4, 14, battery_bar_color);

        // charge in progress -> blink last bar
        if (anim_switch) {
            ILI9341_Draw_Rectangle(x + 12, y, 4, 14, battery_bar_color);
            anim_switch = false;
        } else {
            ILI9341_Draw_Rectangle(x + 12, y, 4, 14, BLACK);
            anim_switch = true;
        }
    } else if (battery_state.chip_state != BATTERY_LOW_BATTERY) {
        // all other cases than LBO (low battery) -> add bars
        if (lvl_perc < 25) {
            battery_bar_color = RED;
        }

        ILI9341_Draw_Rectangle(x, y, 4, 14, battery_bar_color);

        if (lvl_perc > 50) {
            ILI9341_Draw_Rectangle(x + 6, y, 4, 14, battery_bar_color);
        }

        if (lvl_perc > 75) {
            ILI9341_Draw_Rectangle(x + 12, y, 4, 14, battery_bar_color);
        }

        if (battery_state.chip_state == BATTERY_NOT_AVAILABLE) {
            // No battery or standby -> battery crossed out
            ILI9341_Draw_Rectangle(x - 4, y + 5, 26, 2, RED);
        }
    }
}

void Common_DrawInitial() {
    UiState state;

    Common_CleanUp();

    Common_DrawHeader();
    Common_DrawFPS(&state);
    Common_DrawRecordState(&state);
    Common_DrawHeatmap(&state);
    Common_DrawBattery();
}

void Common_StickPressed(void) {
    Navigation_NavigateToById(FRAME_MENU_OVERVIEW);
}

void Common_BatteryStateChanged(BatteryState state) {
    battery_state = state;
    redraw_battery_state = true;
}

void Common_AnimationHandler(void) {
    if (HAL_GetTick() - last_batt_anim_time > 1000 && (
            redraw_battery_state || battery_state.chip_state == BATTERY_CHARGING)) {
        Common_DrawBattery();
        last_batt_anim_time = HAL_GetTick();
        redraw_battery_state = false;
    }
}
