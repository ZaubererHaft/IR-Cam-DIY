#include "file_system.h"

#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "heatmap.h"

static FATFS fs;
static uint8_t bitmap_buffer[32 * 24 * 2 + 66];
static HeatmapFunction TemperatureConverter;

#define CONFIG_FILE_NAME "config.txt"

uint32_t FileSystem_FileExists(const char *file) {
    FILINFO fno;
    const FRESULT res = f_stat(file, &fno);

    if (res == FR_OK && !(fno.fattrib & AM_DIR)) {
        return 1;
    }

    return 0;
}

FRESULT FileSystem_SaveConfig(const Config *config) {
    FIL config_file;
    UINT bw;

    FRESULT res = f_open(&config_file, CONFIG_FILE_NAME, FA_CREATE_ALWAYS | FA_WRITE);

    if (res == FR_OK) {
        char buff[64] = {0};
        int32_t written = snprintf(buff, sizeof(buff), "%lu\n%lu\n%lu\n%.2f\n%.2f\n",
            config->image_counter,
            config->heatmap_index,
            config->autoscale,
            config->tMin,
            config->tMax);

        if (written > 0) {
            res |= f_write(&config_file, buff, strlen(buff), &bw);
            res |= f_close(&config_file);
        } else {
            res = FR_INT_ERR;
        }
    }

    return res;
}


FRESULT FileSystem_CreateNewConfig() {
    Config config_new;
    config_new.image_counter = 0;
    config_new.heatmap_index = 0;

    return FileSystem_SaveConfig(&config_new);
}

uint32_t read_and_convert(FIL *file, uint32_t *out_value) {
    char buff[32] = {0};

    if (f_gets(buff, sizeof(buff), file) != NULL) {
        char *endptr;
        const uint32_t value = strtoul(buff, &endptr, 10);

        if (buff != endptr) {
            *out_value = value;
            return 1;
        }
    }

    return 0;
}

FRESULT FileSystem_ReadConfig(Config *config) {
    FIL config_file;

    FRESULT res = f_open(&config_file, CONFIG_FILE_NAME, FA_READ);
    if (res == FR_OK) {
        if (!read_and_convert(&config_file, &config->image_counter)) {
            res |= FR_INT_ERR;
        }

        if (!read_and_convert(&config_file, &config->heatmap_index)) {
            res |= FR_INT_ERR;
        }

        res |= f_close(&config_file);
    }

    return res;
}

FRESULT create_if_necessary_and_read(Config *out_config) {
    FRESULT res = FR_OK;

    if (!FileSystem_FileExists(CONFIG_FILE_NAME)) {
        res = FileSystem_CreateNewConfig();
    }

    res |= FileSystem_ReadConfig(out_config);
    return res;
}

uint32_t FileSystem_Init(uint8_t *buffer, Config *out_config) {
    FRESULT res = f_mount(&fs, "", 1);

    // File system already exists -> check for config
    if (res == FR_OK) {
        res = create_if_necessary_and_read(out_config);
    } else {
        // no file system available, try to create...
        res = f_mkfs("0:", FM_ANY | FM_SFD, 4096, buffer, 4096);
        if (res == FR_OK) {
            // ... and mount
            res = f_mount(&fs, "", 1);
            if (res == FR_OK) {
                // now again: try to create config
                res = create_if_necessary_and_read(out_config);
            }
        }
    }

    return res == FR_OK;
}

uint32_t FileSystem_WriteBitmap(const float *image, uint32_t size, const char *name) {
    bitmap_buffer[0] = 'B';
    bitmap_buffer[1] = 'M';

    //file size
    bitmap_buffer[2] = (size & 0x00FF) >> 0;
    bitmap_buffer[3] = (size & 0xFF00) >> 8;
    bitmap_buffer[4] = 0;
    bitmap_buffer[5] = 0;

    // reserved
    bitmap_buffer[6] = 0;
    bitmap_buffer[7] = 0;
    bitmap_buffer[8] = 0;
    bitmap_buffer[9] = 0;

    // offset
    bitmap_buffer[10] = 66;
    bitmap_buffer[11] = 0;
    bitmap_buffer[12] = 0;
    bitmap_buffer[13] = 0;

    // -----------------
    // size of info header
    bitmap_buffer[14] = 40;
    bitmap_buffer[15] = 0;
    bitmap_buffer[16] = 0;
    bitmap_buffer[17] = 0;

    // horizontal width
    bitmap_buffer[18] = 32;
    bitmap_buffer[19] = 0;
    bitmap_buffer[20] = 0;
    bitmap_buffer[21] = 0;

    // vertical height
    bitmap_buffer[22] = 0xE8;
    bitmap_buffer[23] = 0xFF;
    bitmap_buffer[24] = 0xFF;
    bitmap_buffer[25] = 0xFF;

    // 1 plane
    bitmap_buffer[26] = 1;
    bitmap_buffer[27] = 0;

    // 16 bits per pixel
    bitmap_buffer[28] = 16;
    bitmap_buffer[29] = 0;

    //compression - BI BITFIELDS
    bitmap_buffer[30] = 3;
    bitmap_buffer[31] = 0;
    bitmap_buffer[32] = 0;
    bitmap_buffer[33] = 0;

    // compressed image size
    bitmap_buffer[34] = 0;
    bitmap_buffer[35] = 0;
    bitmap_buffer[36] = 0;
    bitmap_buffer[37] = 0;

    //pixel per m X
    bitmap_buffer[38] = 0x13;
    bitmap_buffer[39] = 0x0B;
    bitmap_buffer[40] = 0;
    bitmap_buffer[41] = 0;

    //pixel per m Y
    bitmap_buffer[42] = 0x13;
    bitmap_buffer[43] = 0x0B;
    bitmap_buffer[44] = 0;
    bitmap_buffer[45] = 0;

    //Colors used
    bitmap_buffer[46] = 0;
    bitmap_buffer[47] = 0;
    bitmap_buffer[48] = 0;
    bitmap_buffer[49] = 0;

    //All colors are important
    bitmap_buffer[50] = 0;
    bitmap_buffer[51] = 0;
    bitmap_buffer[52] = 0;
    bitmap_buffer[53] = 0;

    // Rot-Maske: 0xF800 (11111000 00000000)
    bitmap_buffer[54] = 0x00;
    bitmap_buffer[55] = 0xF8;
    bitmap_buffer[56] = 0x00;
    bitmap_buffer[57] = 0x00;
    // Grün-Maske: 0x07E0 (00000111 11100000)
    bitmap_buffer[58] = 0xE0;
    bitmap_buffer[59] = 0x07;
    bitmap_buffer[60] = 0x00;
    bitmap_buffer[61] = 0x00;
    // Blau-Maske: 0x001F (00000000 00011111)
    bitmap_buffer[62] = 0x1F;
    bitmap_buffer[63] = 0x00;
    bitmap_buffer[64] = 0x00;
    bitmap_buffer[65] = 0x00;

    for (int i = 0; i < size; ++i) {
        const uint16_t col = TemperatureConverter(image[i]);
        bitmap_buffer[66 + i * 2] = col & 0xFF;
        bitmap_buffer[66 + i * 2 + 1] = col >> 8;
    }

    FIL fil;
    FRESULT res = f_open(&fil, name, FA_CREATE_ALWAYS | FA_WRITE);

    UINT written;
    if (res == FR_OK) {
        res |= f_write(&fil, bitmap_buffer, sizeof(bitmap_buffer), &written);
        res |= f_close(&fil);
    }

    return res == FR_OK;
}

uint32_t FileSystem_WriteConfig(const Config *config) {
    return FileSystem_SaveConfig(config) == FR_OK;
}

void FileSystem_ConfigObserver(Config config) {
  TemperatureConverter = Heatmap_GetByIndex(config.heatmap_index);
}

