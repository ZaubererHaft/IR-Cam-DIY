#include "dialog.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "navigation.h"

#include <stddef.h>


const char *dialog_text = NULL;

void Dialog_DrawInitial(void) {
    uint16_t y = 80;
    uint16_t x = 20;

    ILI9341_Draw_Rectangle(x, y, 220, 60, BLACK);
    ILI9341_Draw_Text("Message", x + 60, y, ORANGE, 2, BLACK);
    y += 16;

    if (dialog_text != NULL) {
        ILI9341_Draw_Text(dialog_text, x, y, WHITE, 1, BLACK);
        y += 16;
    }

    ILI9341_Draw_Text("OK", x + 100, y, BLACK, 2, WHITE);
}

void Dialog_StickPressed(void) {
    Navigation_NavigateBack();
}

void Dialog_SetText(const char *text) {
    dialog_text = text;
}

void Dialog_Close(void) {
    Dialog_StickPressed();
}

