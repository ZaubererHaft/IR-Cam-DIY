#include "gallery.h"
#include "ff.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"

#include <stdbool.h>
#include <string.h>

#include "navigation.h"

#define MARKER_Y_TOP 0
#define MARKER_Y_BOTTOM 1

#define MARKER_X_INDEX_PREV 0
#define MARKER_X_INDEX_DEL 1
#define MARKER_X_INDEX_NEXT 2

#define PREVIEW_PIXEL_SIZE 4

static uint32_t marker_index_x = MARKER_X_INDEX_NEXT;
static uint32_t marker_index_y = MARKER_Y_BOTTOM;

static uint32_t available_files = 0;
static uint32_t curr_image_index = 0;

static char files[1000][12];

static bool string_ends_with(const char *str, const char *suffix) {
    if (!str || !suffix) return false;

    size_t str_len = strlen(str);
    size_t suffix_len = strlen(suffix);

    if (suffix_len > str_len) return false;

    return strcmp(str + str_len - suffix_len, suffix) == 0;
}


static void Gallery_ClearImage() {
    ILI9341_Draw_Rectangle(60, 70, 32 * PREVIEW_PIXEL_SIZE + 40, 24 * PREVIEW_PIXEL_SIZE + 40, BLACK);
}


static void Gallery_DrawCurrentImage() {
    Gallery_ClearImage();

    if (available_files > 0) {
        FIL fil;
        BYTE byte_buffer;
        UINT bytes_read;

        FRESULT fr = f_open(&fil, files[curr_image_index], FA_READ);

        if (fr == FR_OK) {
            fr = f_lseek(&fil, 66);

            uint16_t pixel = 0;
            bool le = true;

            uint16_t row = 0;
            uint16_t col = 0;

            do {
                fr = f_read(&fil, &byte_buffer, 1, &bytes_read);
                if (fr != FR_OK) {
                    break;
                }

                if (le) {
                    pixel |= byte_buffer;
                    le = false;
                } else {
                    pixel |= (byte_buffer << 8);
                    le = true;

                    ILI9341_Draw_Rectangle(60 + col * PREVIEW_PIXEL_SIZE, 70 + PREVIEW_PIXEL_SIZE * row,
                                           PREVIEW_PIXEL_SIZE, PREVIEW_PIXEL_SIZE, pixel);

                    pixel = 0;

                    col++;
                    if (col == 32) {
                        row++;
                        col = 0;
                    }
                }
            } while (bytes_read > 0);

            f_close(&fil);

            ILI9341_Draw_Text(files[curr_image_index], 100, 200, WHITE, 1, BLACK);
        }
    }
    else {
        ILI9341_Draw_Text("No images found", 100, 120, WHITE, 1, BLACK);
        ILI9341_Draw_Text("Why not make one?", 95, 130, WHITE, 1, BLACK);
    }
}


static void Gallery_ReadAllFiles() {
    DIR current_dir;
    FILINFO current_file;
    FRESULT fresult = FR_OK;

    available_files = 0;

    fresult = f_opendir(&current_dir, "");

    if (fresult == FR_OK) {
        for (;;) {
            fresult = f_readdir(&current_dir, &current_file);
            if (fresult != FR_OK) {
                break;
            }

            if (current_file.fname[0] == 0) {
                break;
            }

            if (current_file.fname[0] == '.') {
                continue;
            }

            if (current_file.fattrib & AM_DIR) {
                continue;
            }

            if (string_ends_with(current_file.fname, ".bmp")) {
                strcpy(files[available_files], current_file.fname);
                available_files++;
            }
        }

        f_closedir(&current_dir);
    }
}

static void Gallery_DeleteCurrentImage() {
    if (available_files > 0) {
        FRESULT fr = f_unlink(files[curr_image_index]);

        if (fr == FR_OK) {
            available_files--;

            if (available_files > 0) {
                curr_image_index = available_files - 1;
            }

            Gallery_ReadAllFiles();
            Gallery_DrawCurrentImage();
        }
    }
}

static void Draw_Element(const char *text, uint32_t y_pos, uint32_t x_pos, uint32_t index_y, uint32_t index_x) {
    uint16_t color_foreground = WHITE;
    uint16_t color_background = BLACK;

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

    Draw_Element(" x", 40, 220, MARKER_Y_TOP, MARKER_X_INDEX_NEXT);
}

void Gallery_DrawInitial() {
    ILI9341_Draw_Text("Gallery", 80, 40, WHITE, 2, BLACK);

    Gallery_ReadAllFiles();
    Gallery_RedrawNavigationBar();
    Gallery_DrawCurrentImage();
}

void Gallery_StickRightMoved(void) {
    if (marker_index_x < MARKER_X_INDEX_NEXT && marker_index_y == MARKER_Y_BOTTOM) {
        marker_index_x++;
        Gallery_RedrawNavigationBar();
    }
}

void Gallery_StickLeftMoved(void) {
    if (marker_index_x > MARKER_X_INDEX_PREV && marker_index_y == MARKER_Y_BOTTOM) {
        marker_index_x--;
        Gallery_RedrawNavigationBar();
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
            if (available_files > 0) {
                curr_image_index = (curr_image_index + 1) % available_files;

                Gallery_DrawCurrentImage();
            }
        } else if (marker_index_x == MARKER_X_INDEX_PREV) {
            if (available_files > 0) {
                if (curr_image_index <= 0) {
                    curr_image_index = available_files - 1;
                } else {
                    curr_image_index = curr_image_index - 1;
                }

                Gallery_DrawCurrentImage();
            }
        } else if (marker_index_x == MARKER_X_INDEX_DEL) {
            Gallery_DeleteCurrentImage();
        }
    }
}
