#include "gallery.h"

#include <string.h>

#include "ff.h"
#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"


static bool string_ends_with(const char *str, const char *suffix) {
    if (!str || !suffix) return false;

    size_t str_len = strlen(str);
    size_t suffix_len = strlen(suffix);

    if (suffix_len > str_len) return false;

    return strcmp(str + str_len - suffix_len, suffix) == 0;
}

void Gallery_Draw(UiState *state) {
    FRESULT res;
    DIR dir;
    FILINFO fno;

    res = f_opendir(&dir, "");
    if (res == FR_OK) {
        int x = 80;
        int y = 40;

        ILI9341_Draw_Text("Gallery", x, y, WHITE, 2, BLACK);
        y += 20;

        for (;;) {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;

            if (fno.fname[0] == '.') continue;

            if (!(fno.fattrib & AM_DIR)) {
                if (string_ends_with(fno.fname, ".bmp")) {
                    ILI9341_Draw_Text(fno.fname, x, y, WHITE, 2, BLACK);
                    y += 20;
                }
            }
        }
        f_closedir(&dir);
    }

}
