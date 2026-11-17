#include "settings_advanced.h"

#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "navigation.h"

static uint16_t menu_cursor_Y;

typedef enum MENU_ {
    MENU_SAVE,
    MENU_BACK
} MENU;

static void DrawMenuLine(const char *text, uint16_t x, uint16_t *y, uint16_t line) {
    *y += 16;

    uint16_t color_foreground;
    uint16_t color_background;

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


static void SettingsAdvanced_Redraw(void) {
    uint16_t y = 40;
    uint16_t x = 15;

    ILI9341_Draw_Rectangle(x, y, 220, 16, BLACK);
    ILI9341_Draw_Text(" Settings (Adv.)", 30, y, ORANGE, 2, BLACK);

    DrawMenuLine(" Save ", x, &y, MENU_SAVE);
    DrawMenuLine(" Back ", x, &y, MENU_BACK);
}

void SettingsAdvanced_DrawInitial(void) {
    SettingsAdvanced_Redraw();
}

void SettingsAdvanced_StickMovedUp(void) {
    if (menu_cursor_Y <= 0) {
        menu_cursor_Y = MENU_BACK;
    } else {
        menu_cursor_Y--;
    }

    SettingsAdvanced_Redraw();
}

void SettingsAdvanced_StickMovedDown(void) {
    menu_cursor_Y = (menu_cursor_Y + 1) % (MENU_BACK + 1);
    SettingsAdvanced_Redraw();
}

void SettingsAdvanced_StickMovedRight(void) {
}

void SettingsAdvanced_StickMovedLeft(void) {
}

void SettingsAdvanced_StickPressed(void) {
    if (menu_cursor_Y == MENU_BACK) {
        Navigation_NavigateBack();
    }
}

void SettingsAdvanced_ConfigChanged(Config config) {
}
