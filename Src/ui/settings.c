#include "settings.h"

#include <stdint.h>
#include <stdio.h>

#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "interface_callback.h"
#include "navigation.h"

static uint16_t menu_cursor_Y;
static uint16_t requested_heatmap;
static Config curr_config;

typedef enum MENU_ {
    MENU_HEATMAP,
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

void Settings_Redraw(void) {
    uint16_t y = 80;
    uint16_t x = 15;

    ILI9341_Draw_Rectangle(x, y, 220, 16, BLACK);
    ILI9341_Draw_Text(" Settings", 60, y, ORANGE, 2, BLACK);

    char selected_heatmap[40];
    snprintf(selected_heatmap, sizeof(selected_heatmap), " Heatmap: %s", Heatmap_GetNameByIndex(requested_heatmap));
    DrawMenuLine(selected_heatmap, x, &y, MENU_HEATMAP);
    DrawMenuLine(" Save ", x, &y, MENU_SAVE);
    DrawMenuLine(" Back ", x, &y, MENU_BACK);
}


void Settings_DrawInitial(void) {
    requested_heatmap = curr_config.heatmap_index;
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

void Settings_StickPressed(void) {
    if (menu_cursor_Y == MENU_HEATMAP) {
        requested_heatmap = (requested_heatmap + 1) % Heatmap_GetSize();
        Settings_Redraw();
    }
    else if (menu_cursor_Y == MENU_SAVE) {
        InterfaceCallback_RequestNewHeatmap(requested_heatmap);
        Navigation_NavigateBack();
    }
    else if (menu_cursor_Y == MENU_BACK) {
        Navigation_NavigateBack();
    }
}

void Settings_ConfigChanged(Config config) {
    requested_heatmap = config.heatmap_index;

    curr_config = config;
}
