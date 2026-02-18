#include "file_system.h"
#include "ff.h"
#include "heatmap.h"

static FATFS fs;
static uint8_t buffer[(32 * 24 * 2) + 14 + 40];

int32_t FileSystem_Init() {
    FRESULT res;
    return f_mount(&fs, "", 0);
}

int32_t FileSystem_WriteBitmap(float *image, uint32_t size) {

    buffer[0] = 'B';
    buffer[1] = 'M';

    //file size
    buffer[2] = (size & 0x00FF) >> 0;
    buffer[3] = (size & 0xFF00) >> 8;
    buffer[4] = 0;
    buffer[5] = 0;

    // reserved
    buffer[6] = 0;
    buffer[7] = 0;
    buffer[8] = 0;
    buffer[9] = 0;

    // offset
    buffer[10] = 66;
    buffer[11] = 0;
    buffer[12] = 0;
    buffer[13] = 0;

    // -----------------
    // size of info header
    buffer[14] = 40;
    buffer[15] = 0;
    buffer[16] = 0;
    buffer[17] = 0;

    // horizontal width
    buffer[18] = 32;
    buffer[19] = 0;
    buffer[20] = 0;
    buffer[21] = 0;

    // vertical height
    buffer[22] = 24;
    buffer[23] = 0;
    buffer[24] = 0;
    buffer[25] = 0;

    // 1 plane
    buffer[26] = 1;
    buffer[27] = 0;

    // 16 bits per pixel
    buffer[28] = 16;
    buffer[29] = 0;

    //compression - BI BITFIELDS
    buffer[30] = 3;
    buffer[31] = 0;
    buffer[32] = 0;
    buffer[33] = 0;

    // compressed image size
    buffer[34] = 0;
    buffer[35] = 0;
    buffer[36] = 0;
    buffer[37] = 0;

    //pixel per m X
    buffer[38] = 0x13;
    buffer[39] = 0x0B;
    buffer[40] = 0;
    buffer[41] = 0;

    //pixel per m Y
    buffer[42] = 0x13;
    buffer[43] = 0x0B;
    buffer[44] = 0;
    buffer[45] = 0;

    //Colors used
    buffer[46] = 0;
    buffer[47] = 0;
    buffer[48] = 0;
    buffer[49] = 0;

    //All colors are important
    buffer[50] = 0;
    buffer[51] = 0;
    buffer[52] = 0;
    buffer[53] = 0;

    // Rot-Maske: 0xF800 (11111000 00000000)
    buffer[54] = 0x00;
    buffer[55] = 0xF8;
    buffer[56] = 0x00;
    buffer[57] = 0x00;
    // Grün-Maske: 0x07E0 (00000111 11100000)
    buffer[58] = 0xE0;
    buffer[59] = 0x07;
    buffer[60] = 0x00;
    buffer[61] = 0x00;
    // Blau-Maske: 0x001F (00000000 00011111)
    buffer[62] = 0x1F;
    buffer[63] = 0x00;
    buffer[64] = 0x00;
    buffer[65] = 0x00;

    for (int i = 0; i < size; ++i) {
        uint16_t col = TempConverter(image[i]);
        buffer[66 + i * 2] = col & 0xFF;
        buffer[67 + (i + 1) * 2] = col >> 8;
    }

    FIL fil;
    FRESULT res = f_open(&fil, "test.bmp", FA_CREATE_ALWAYS | FA_WRITE);

    UINT written;
    if (res == FR_OK) {
      f_write(&fil, buffer, sizeof(buffer), &written);
      f_close(&fil);
    }

    return 0;
}
