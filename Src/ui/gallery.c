#include "gallery.h"
#include "ff.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "file_system.h"
#include "navigation.h"
#include "ui_constants.h"

#include <string.h>

#define MARKER_Y_TOP 0
#define MARKER_Y_BOTTOM 1

#define MARKER_X_INDEX_PREV 0
#define MARKER_X_INDEX_DEL 1
#define MARKER_X_INDEX_NEXT 2


static uint32_t marker_index_x = MARKER_X_INDEX_NEXT;
static uint32_t marker_index_y = MARKER_Y_BOTTOM;

static char *curr_file_name = NULL;
static uint16_t line_buffer[256];

static void Gallery_ClearImage() {
    ILI9341_Draw_Rectangle(IR_IMAGE_BEGIN_X, IR_IMAGE_BEGIN_X, 32 * 4 + 40, 24 * 4 + 45, BLACK);
}

static void Draw_Element(const char *text, uint32_t y_pos, uint32_t x_pos, uint32_t index_y, uint32_t index_x) {
    uint16_t color_foreground;
    uint16_t color_background;

    if (marker_index_x == index_x && marker_index_y == index_y) {
        color_foreground = BLACK;
        color_background = WHITE;
    } else {
        color_foreground = WHITE;
        color_background = BLACK;
    }

    ILI9341_Draw_Text(text, x_pos, y_pos, color_foreground, 2, color_background);
}


static void Gallery_RedrawNavigationBar() {
    Draw_Element(" <", 220, 20, MARKER_Y_BOTTOM, MARKER_X_INDEX_PREV);
    Draw_Element(" Del ", 220, 120, MARKER_Y_BOTTOM, MARKER_X_INDEX_DEL);
    Draw_Element(" >", 220, 220, MARKER_Y_BOTTOM, MARKER_X_INDEX_NEXT);

    Draw_Element("x", 30, 230, MARKER_Y_TOP, MARKER_X_INDEX_NEXT);
}


static void Gallery_DrawCurrentImage() {
    Gallery_ClearImage();

    if (FileSystem_AvailableFiles() > 0) {
        FIL fil;

        if (f_open(&fil, curr_file_name, FA_READ) == FR_OK) {
            if (f_lseek(&fil, 66) == FR_OK) {
                ILI9341_Set_Address(
                    IR_IMAGE_BEGIN_X,
                    IR_IMAGE_BEGIN_Y,
                    IR_IMAGE_BEGIN_X + 256 - 1,
                    IR_IMAGE_BEGIN_Y + 192 - 1
                );

                int32_t bytes_left = 256 * 192 * 2;
                while (bytes_left > 0) {
                    UINT bytes_read = 0;
                    UINT to_read = bytes_left < sizeof(line_buffer) ? bytes_left : sizeof(line_buffer);

                    if (f_read(&fil, line_buffer, to_read, &bytes_read) != FR_OK) {
                        Error_Handler();
                    }
                    if (bytes_read == 0) {
                        break;
                    }
                    for (uint32_t i = 0; i < bytes_read / 2; i++) {
                        line_buffer[i] = __REV16(line_buffer[i]);
                    }
                    ILI9341_Send_Data_Block((uint8_t *) line_buffer, bytes_read);
                    bytes_left -= bytes_read;
                }
                ILI9341_Draw_Text(curr_file_name, 120, 200, WHITE, 1, BLACK);
            }

            f_close(&fil);
        }
    } else {
        ILI9341_Draw_Text("No images found", 100, 120, WHITE, 1, BLACK);
        ILI9341_Draw_Text("Why not make one?", 95, 130, WHITE, 1, BLACK);
    }

    Gallery_RedrawNavigationBar();
}


static void Gallery_DeleteCurrentImage() {
    if (FileSystem_AvailableFiles() > 0) {
        if (FileSystem_DeleteBitmapAndRaw(curr_file_name)) {
            if (FileSystem_AvailableFiles() > 0) {
                curr_file_name = FileSystem_NextBitmap();
            }

            Gallery_DrawCurrentImage();
        } else {
            Error_Handler();
        }
    }
}

void Gallery_DrawInitial() {
    ILI9341_Draw_Rectangle(00, 20, 260, 210, BLACK);
    ILI9341_Draw_Text("Gallery", 80, 40, ORANGE, 2, BLACK);

    if (curr_file_name == NULL) {
        curr_file_name = FileSystem_NextBitmap();
    }

    Gallery_RedrawNavigationBar();
    Gallery_DrawCurrentImage();
}

static void Gallery_DrawNextImage() {
    if (FileSystem_AvailableFiles() > 0) {
        curr_file_name = FileSystem_NextBitmap();
        Gallery_DrawCurrentImage();
    }
}

static void Gallery_DrawPreviousImage() {
    if (FileSystem_AvailableFiles() > 0) {
        curr_file_name = FileSystem_PreviousBitmap();
        Gallery_DrawCurrentImage();
    }
}

void Gallery_StickRightMoved(void) {
    if (marker_index_x < MARKER_X_INDEX_NEXT && marker_index_y == MARKER_Y_BOTTOM) {
        marker_index_x++;
        Gallery_RedrawNavigationBar();
    } else if (marker_index_x == MARKER_X_INDEX_NEXT && marker_index_y == MARKER_Y_BOTTOM) {
        Gallery_DrawNextImage();
    }
}

void Gallery_StickLeftMoved(void) {
    if (marker_index_x > MARKER_X_INDEX_PREV && marker_index_y == MARKER_Y_BOTTOM) {
        marker_index_x--;
        Gallery_RedrawNavigationBar();
    } else if (marker_index_x == MARKER_X_INDEX_PREV && marker_index_y == MARKER_Y_BOTTOM) {
        Gallery_DrawPreviousImage();
    }
}

void Gallery_StickUpMoved(void) {
    marker_index_y = MARKER_Y_TOP;
    marker_index_x = MARKER_X_INDEX_NEXT;
    Gallery_RedrawNavigationBar();
}

void Gallery_StickDownMoved(void) {
    marker_index_y = MARKER_Y_BOTTOM;
    Gallery_RedrawNavigationBar();
}


void Gallery_StickPressed(void) {
    if (marker_index_y == MARKER_Y_TOP) {
        Navigation_NavigateBack();
    } else {
        if (marker_index_x == MARKER_X_INDEX_NEXT) {
            Gallery_DrawNextImage();
        } else if (marker_index_x == MARKER_X_INDEX_PREV) {
            Gallery_DrawPreviousImage();
        } else if (marker_index_x == MARKER_X_INDEX_DEL) {
            Gallery_DeleteCurrentImage();
        }
    }
}
