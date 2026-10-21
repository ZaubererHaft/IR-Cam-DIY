#include "menu.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"

#include <stdint.h>

#include "navigation.h"

typedef enum MENU_ {
    MENU_SAVE_IMAGE,
    MENU_GALLERY,
    MENU_SETTINGS,
    MENU_EXIT
} MENU;

static uint32_t menu_cursor_Y = 0;

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

    ILI9341_Draw_Rectangle(x, *y, 160, 16, color_background);
    ILI9341_Draw_Text(text, x, *y, color_foreground, 2, color_background);
}

static void Menu_RedrawMenu(void) {
    uint16_t y = 80;
    uint16_t x = 50;

    ILI9341_Draw_Rectangle(x, y, 160, 16, BLACK);
    ILI9341_Draw_Text(" Menu", y, y, WHITE, 2, BLACK);

    DrawMenuLine(" Save Image", x, &y, MENU_SAVE_IMAGE);
    DrawMenuLine(" Gallery", x, &y, MENU_GALLERY);
    DrawMenuLine(" Settings", x, &y, MENU_SETTINGS);
    DrawMenuLine(" Exit", x, &y, MENU_EXIT);
}

void Menu_DrawInitial() {
    menu_cursor_Y = 0;
    Menu_RedrawMenu();
}

void Menu_StickMovedUp(void) {
    if (menu_cursor_Y <= 0) {
        menu_cursor_Y = MENU_EXIT;
    } else {
        menu_cursor_Y--;
    }

    Menu_RedrawMenu();
}

void Menu_StickMovedDown(void) {
    menu_cursor_Y = (menu_cursor_Y + 1) % (MENU_EXIT + 1);
    Menu_RedrawMenu();
}

void Menu_StickPressed(void) {
    if (menu_cursor_Y == MENU_EXIT) {
        Navigation_NavigateBack();
    } else if (menu_cursor_Y == MENU_GALLERY) {
        Navigation_NavigateToById(FRAME_GALLERY);
    }
}
