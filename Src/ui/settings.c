#include "settings.h"

#include <stdint.h>
#include <stdio.h>

#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "interface_callback.h"
#include "navigation.h"

static uint16_t menu_cursor_Y;

static Config requested_config;
static Config curr_config;

typedef enum MENU_ {
    MENU_HEATMAP,
    MENU_MIN,
    MENU_MAX,
    MENU_UPSCALING,
    MENU_AUTOSCALE,
    MENU_SAVE,
    MENU_BACK
} MENU;

static void DrawMenuLine(const char *text, uint16_t x, uint16_t *y, uint16_t line) {
    *y += 16;

    uint16_t color_foreground = WHITE;
    uint16_t color_background = BLACK;

    if (menu_cursor_Y == line) {
        color_foreground = BLACK;
        color_background = WHITE;
    } else {
        color_foreground = WHITE;
        color_background = BLACK;
    }

    ILI9341_Draw_Rectangle(x, *y, 220, 16, color_background);
    ILI9341_Draw_Text(text, x, *y, color_foreground, 2, color_background);
}

static void Settings_Redraw(void) {
    uint16_t y = 40;
    uint16_t x = 15;

    ILI9341_Draw_Rectangle(x, y, 220, 16, BLACK);
    ILI9341_Draw_Text(" Settings", 60, y, ORANGE, 2, BLACK);

    char buffer[40];
    snprintf(buffer, sizeof(buffer), " Heatmap: %s", Heatmap_GetNameByIndex(requested_config.heatmap_index));
    DrawMenuLine(buffer, x, &y, MENU_HEATMAP);

    snprintf(buffer, sizeof(buffer), " Temp min: %.2f", requested_config.tMin);
    DrawMenuLine(buffer, x, &y, MENU_MIN);

    snprintf(buffer, sizeof(buffer), " Temp max: %.2f", requested_config.tMax);
    DrawMenuLine(buffer, x, &y, MENU_MAX);

    snprintf(buffer, sizeof(buffer), " Upscaling: %lu", requested_config.upscaling_factor);
    DrawMenuLine(buffer, x, &y, MENU_UPSCALING);

    snprintf(buffer, sizeof(buffer), " Autoscale: %lu", requested_config.autoscale);
    DrawMenuLine(buffer, x, &y, MENU_AUTOSCALE);

    DrawMenuLine(" Save ", x, &y, MENU_SAVE);
    DrawMenuLine(" Back ", x, &y, MENU_BACK);
}


void Settings_DrawInitial(void) {
    requested_config = curr_config;

    Settings_Redraw();
}

void Settings_StickMovedUp(void) {
    if (menu_cursor_Y <= 0) {
        menu_cursor_Y = MENU_BACK;
    } else {
        menu_cursor_Y--;
    }

    Settings_Redraw();
}

void Settings_StickMovedDown(void) {
    menu_cursor_Y = (menu_cursor_Y + 1) % (MENU_BACK + 1);
    Settings_Redraw();
}

void Settings_StickMovedRight(void) {
    if (menu_cursor_Y == MENU_HEATMAP) {
        requested_config.heatmap_index = (requested_config.heatmap_index + 1) % Heatmap_GetSize();
        Settings_Redraw();
    } else if (menu_cursor_Y == MENU_MIN) {
        if (requested_config.tMin + 1.0f <= requested_config.tMax) {
            requested_config.tMin++;
            Settings_Redraw();
        }
    } else if (menu_cursor_Y == MENU_MAX) {
        if (requested_config.tMax + 1.0f <= TEMPERATURE_MAX_VALUE) {
            requested_config.tMax++;
            Settings_Redraw();
        }
    } else if (menu_cursor_Y == MENU_UPSCALING) {
        if (requested_config.upscaling_factor < 8) {
            requested_config.upscaling_factor *= 2;
            Settings_Redraw();
        }
    } else if (menu_cursor_Y == MENU_AUTOSCALE) {
        requested_config.autoscale = !requested_config.autoscale;
        Settings_Redraw();
    }
}

void Settings_StickMovedLeft(void) {
    if (menu_cursor_Y == MENU_HEATMAP) {
        requested_config.heatmap_index = requested_config.heatmap_index <= 0
                                             ? Heatmap_GetSize() - 1
                                             : requested_config.heatmap_index - 1;
        Settings_Redraw();
    } else if (menu_cursor_Y == MENU_MIN) {
        if (requested_config.tMin - 1.0f >= TEMPERATURE_MIN_VALUE) {
            requested_config.tMin--;
            Settings_Redraw();
        }
    } else if (menu_cursor_Y == MENU_MAX) {
        if (requested_config.tMax - 1.0f >= requested_config.tMin) {
            requested_config.tMax--;
            Settings_Redraw();
        }
    } else if (menu_cursor_Y == MENU_UPSCALING) {
        if (requested_config.upscaling_factor > 1) {
            requested_config.upscaling_factor /= 2;
            Settings_Redraw();
        }
    } else if (menu_cursor_Y == MENU_AUTOSCALE) {
        requested_config.autoscale = !requested_config.autoscale;
        Settings_Redraw();
    }
}

void Settings_StickPressed(void) {
    if (menu_cursor_Y == MENU_SAVE) {
        InterfaceCallback_RequestNewSettings(requested_config);
        Navigation_NavigateBack();
    } else if (menu_cursor_Y == MENU_BACK) {
        Navigation_NavigateBack();
    }
}

void Settings_ConfigChanged(Config config) {
    requested_config = config;
    curr_config = config;
}
