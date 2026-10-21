#include "common.h"
#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


static const uint16_t max_lvl = 3583;
static GPIO_PinState old_stat1;
static GPIO_PinState old_stat2;
static GPIO_PinState old_npg;
static bool initial_draw_battery = 1;
static uint16_t battery_level = max_lvl;
static uint16_t old_battery_level = 0;
static bool last_anim_charge = 1;

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
    //if (state->current_frame == FRAME_DEFAULT) {
      //  ILI9341_Draw_Filled_Circle(304, 12, 8, RED);
    //} else
    {
        ILI9341_Draw_Rectangle(300, 4, 4, 16, LIGHTGREY);
        ILI9341_Draw_Rectangle(306, 4, 4, 16, LIGHTGREY);
    }
}

static void Common_DrawBattery(void) {
    GPIO_PinState stat1 = !HAL_GPIO_ReadPin(BATTCHARGING_GPIO_Port, BATTCHARGING_Pin);
    GPIO_PinState stat2 = !HAL_GPIO_ReadPin(BATTFULL_GPIO_Port, BATTFULL_Pin);
    GPIO_PinState npg = !HAL_GPIO_ReadPin(EXT_POWER_GPIO_Port, EXT_POWER_Pin);

    if (initial_draw_battery || stat1 != old_stat1 || stat2 != old_stat2 || old_npg != npg || abs(
            battery_level - old_battery_level) >= 50 || (
            stat1 && !stat2 && npg)) {

        initial_draw_battery = false;
        old_stat1 = stat1;
        old_stat2 = stat2;
        old_npg = npg;

        uint32_t x = 260;
        uint32_t y = 3;

        // Battery frame
        uint16_t battery_bar_color = WHITE;
        if (!stat1 && stat2 && npg) {
            //charging complete -> green
            battery_bar_color = GREEN;
        }

        // ca. 4.2 V Ladeschlusspannung
        // spannungsteiler 100 - 220 -> max 2.88 V Spannung
        // -> 0.875 Maximal von 3.3 V erreichbar
        // -> 0.875 * 4096 = 3583 sind 100 % batteriespannung
        char buff[20] = {};

        // Battery level in numbers
        uint32_t lvl_perc = (uint32_t) ((float) battery_level / (float) max_lvl * 100.0f);
        snprintf(buff, sizeof(buff), "%lu%%", lvl_perc);
        ILI9341_Draw_Text(buff, x - 55, 0, WHITE, 2, BLACK);
        old_battery_level = battery_level;

        // Battery frame
        ILI9341_Draw_Hollow_Rectangle_Coord(x - 3, 0, x + 18, y + 14 + 2, WHITE);
        ILI9341_Draw_Rectangle(x + 18, y + 1, 3, 10, WHITE);

        if (stat1 && !stat2 && npg) {
            // Two bars
            ILI9341_Draw_Rectangle(x, y, 4, 14, battery_bar_color);
            ILI9341_Draw_Rectangle(x + 6, y, 4, 14, battery_bar_color);

            // charge in progress -> blink last bar
            if (last_anim_charge) {
                ILI9341_Draw_Rectangle(x + 12, y, 4, 14, battery_bar_color);
                last_anim_charge = false;
            } else {
                ILI9341_Draw_Rectangle(x + 12, y, 4, 14, BLACK);
                last_anim_charge = true;
            }
        } else if (!(stat1 && !stat2 && !npg)) {
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

            if (!stat1 && !stat2 && npg) {
                // No battery or standby -> battery crossed out
                ILI9341_Draw_Rectangle(x - 4, y + 5, 26, 2, RED);
            }
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
