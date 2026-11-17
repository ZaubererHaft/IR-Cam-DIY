#include "file_system.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "heatmap.h"


#define CONFIG_FILE_NAME "config.txt"
#define BUFFER_SIZE 512

static FATFS fs;
static char buffer[BUFFER_SIZE];

static uint32_t available_bmps = 0;

static char first_bmp[32] = {0};
static char prev_bmp[32] = {0};
static char curr_bitmap[32] = {0};
static char next_bmp[32] = {0};
static char last_bmp[32] = {0};

typedef uint16_t (*ProcessFunction)(int16_t);

static uint32_t FileSystem_FileExists(const char *file) {
    FILINFO fno;
    const FRESULT res = f_stat(file, &fno);

    if (res == FR_OK && !(fno.fattrib & AM_DIR)) {
        return 1;
    }

    return 0;
}

static FRESULT FileSystem_SaveConfig(const Config *config) {
    FIL config_file;
    UINT bw;

    FRESULT res = f_open(&config_file, CONFIG_FILE_NAME, FA_CREATE_ALWAYS | FA_WRITE);

    if (res == FR_OK) {
        int32_t written = snprintf(buffer, BUFFER_SIZE, "%u\n%lu\n%lu\n%.2f\n%.2f\n%lu\n",
                                   config->image_counter,
                                   config->heatmap_index,
                                   config->autoscale,
                                   config->tMin,
                                   config->tMax,
                                   config->upscaling_factor);

        if (written > 0) {
            res |= f_write(&config_file, buffer, written, &bw);
            res |= f_close(&config_file);
        } else {
            res = FR_INT_ERR;
        }
    }

    return res;
}


static FRESULT FileSystem_CreateNewConfig() {
    Config config_new;
    config_new.image_counter = 0;
    config_new.heatmap_index = 0;
    config_new.tMin = 20;
    config_new.tMax = 35;
    config_new.autoscale = 0;
    config_new.upscaling_factor = 4;

    return FileSystem_SaveConfig(&config_new);
}

static bool read_and_convert_u16(FIL *file, uint16_t *out_value) {
    if (f_gets(buffer, BUFFER_SIZE, file) != NULL) {
        char *endptr;
        const uint16_t value = strtoul(buffer, &endptr, 10);

        if (buffer != endptr) {
            *out_value = value;
            return true;
        }
    }

    return false;
}

static bool read_and_convert_u32(FIL *file, uint32_t *out_value) {
    if (f_gets(buffer, BUFFER_SIZE, file) != NULL) {
        char *endptr;
        const uint32_t value = strtoul(buffer, &endptr, 10);

        if (buffer != endptr) {
            *out_value = value;
            return true;
        }
    }

    return false;
}

static bool read_and_convert_f32(FIL *file, float *out_value) {
    if (f_gets(buffer, BUFFER_SIZE, file) != NULL) {
        char *endptr;
        const float value = strtof(buffer, &endptr);

        if (buffer != endptr) {
            *out_value = value;
            return true;
        }
    }

    return false;
}

static FRESULT FileSystem_ReadConfig(Config *config) {
    FIL config_file;

    FRESULT res = f_open(&config_file, CONFIG_FILE_NAME, FA_READ);
    if (res == FR_OK) {
        if (!read_and_convert_u16(&config_file, &config->image_counter)) {
            res |= FR_INT_ERR;
        }

        if (!read_and_convert_u32(&config_file, &config->heatmap_index)) {
            res |= FR_INT_ERR;
        }

        if (!read_and_convert_u32(&config_file, &config->autoscale)) {
            res |= FR_INT_ERR;
        }

        if (!read_and_convert_f32(&config_file, &config->tMin)) {
            res |= FR_INT_ERR;
        }

        if (!read_and_convert_f32(&config_file, &config->tMax)) {
            res |= FR_INT_ERR;
        }

        if (!read_and_convert_u32(&config_file, &config->upscaling_factor)) {
            res |= FR_INT_ERR;
        }

        res |= f_close(&config_file);
    }

    return res;
}

static FRESULT create_if_necessary_and_read(Config *out_config) {
    FRESULT res = FR_OK;

    if (!FileSystem_FileExists(CONFIG_FILE_NAME)) {
        res = FileSystem_CreateNewConfig();
    }

    res |= FileSystem_ReadConfig(out_config);
    return res;
}

static bool string_ends_with(const char *str, const char *suffix) {
    if (!str || !suffix) return false;

    size_t str_len = strlen(str);
    size_t suffix_len = strlen(suffix);

    if (suffix_len > str_len) return false;

    return strcmp(str + str_len - suffix_len, suffix) == 0;
}


static bool rebuild_curr_bmp_window() {
    DIR current_dir;
    FILINFO current_file;

    if (f_opendir(&current_dir, "") != FR_OK) {
        return false;
    }

    bool first = true;
    bool skipper = false;
    bool window_built = false;

    available_bmps = 0;

    for (;;) {
        FRESULT fresult = f_readdir(&current_dir, &current_file);
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
            available_bmps++;

            if (first) {
                strcpy(first_bmp, current_file.fname);
                first = false;
            }

            if (!window_built) {
                // "middle" file -> next file will be next file
                if (strlen(curr_bitmap) <= 0 || strcmp(curr_bitmap, current_file.fname) == 0) {
                    skipper = true;
                } else {
                    if (skipper) {
                        // next has been set, window found
                        strcpy(next_bmp, current_file.fname);
                        window_built = true;
                    } else {
                        // middle was not yet found, this is previous
                        strcpy(prev_bmp, current_file.fname);
                    }
                }
            }

            strcpy(last_bmp, current_file.fname);
        }
    }

    if (f_closedir(&current_dir)) {
        return false;
    }

    return true;
}

bool FileSystem_Init(uint8_t *tmp_buffer, Config *out_config) {
    FRESULT res = f_mount(&fs, "", 1);

    // File system already exists -> check for config
    if (res == FR_OK) {
        res = create_if_necessary_and_read(out_config);
    } else {
        // no file system available, try to create...
        res = f_mkfs("0:", FM_ANY | FM_SFD, 4096, tmp_buffer, 4096);
        if (res == FR_OK) {
            // ... and mount
            res = f_mount(&fs, "", 1);
            if (res == FR_OK) {
                // now again: try to create config
                res = create_if_necessary_and_read(out_config);
            }
        }
    }

    if (res == FR_OK) {
        rebuild_curr_bmp_window();
        return true;
    }

    return false;
}


bool FileSystem_WriteConfig(const Config *config) {
    return FileSystem_SaveConfig(config) == FR_OK;
}


char *FileSystem_NextBitmap(void) {
    if (rebuild_curr_bmp_window()) {
        if (strlen(next_bmp) <= 0 || strcmp(next_bmp, curr_bitmap) == 0) {
            strcpy(curr_bitmap, first_bmp);
        } else {
            strcpy(curr_bitmap, next_bmp);
        }
        return curr_bitmap;
    }
    return NULL;
}

char *FileSystem_PreviousBitmap(void) {
    if (rebuild_curr_bmp_window()) {
        if (strlen(prev_bmp) <= 0 || strcmp(prev_bmp, curr_bitmap) == 0) {
            strcpy(curr_bitmap, last_bmp);
        } else {
            strcpy(curr_bitmap, prev_bmp);
        }
        return curr_bitmap;
    }
    return NULL;
}


static uint16_t temp_to_color(int16_t temp) {
    return Heatmap_TempScaledToRGB565(temp).raw;
}

static uint16_t temp_identity_cast(int16_t temp) {
    return (uint16_t) temp;
}

static bool f_write_blockwise(FIL *fil, uint32_t size, const int16_t *data, ProcessFunction function) {
    uint32_t values_left = size;
    uint32_t buff_size_in_values = BUFFER_SIZE / 2;
    uint32_t value_pos = 0;

    while (values_left > 0) {
        const uint32_t pixels_to_write = values_left >= buff_size_in_values ? buff_size_in_values : values_left;

        for (uint32_t j = 0; j < pixels_to_write; ++j) {
            const uint16_t temp = function(data[value_pos + j]);
            buffer[j * 2] = temp & 0xFF;
            buffer[j * 2 + 1] = temp >> 8;
        }

        value_pos += pixels_to_write;

        UINT written;
        const FRESULT res = f_write(fil, buffer, pixels_to_write * 2, &written);
        if (res != FR_OK || written != pixels_to_write * 2) {
            return false;
        }

        values_left -= pixels_to_write;
    }

    return true;
}


bool FileSystem_WriteBitmap(const int16_t *image_scaled, uint32_t size, const char *name) {
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
    buffer[22] = 0xE8;
    buffer[23] = 0xFF;
    buffer[24] = 0xFF;
    buffer[25] = 0xFF;

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

    FIL fil;
    FRESULT res = f_open(&fil, name, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        return false;
    }

    UINT written;
    //write header
    res = f_write(&fil, buffer, 66, &written);
    if (res != FR_OK || written != 66) {
        (void) f_close(&fil);
        return false;
    }

    // write data
    bool ret = f_write_blockwise(&fil, size, image_scaled, &temp_to_color);
    ret |= f_close(&fil) == FR_OK;
    ret |= rebuild_curr_bmp_window();

    return ret;
}


bool FileSystem_WriteRaw(const int16_t *image_raw, uint32_t size, const char *name) {
    FIL fil;
    FRESULT res = f_open(&fil, name, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        return false;
    }
    bool ret = f_write_blockwise(&fil, size, image_raw, &temp_identity_cast);
    ret |= f_close(&fil) == FR_OK;
    ret |= rebuild_curr_bmp_window();

    return ret;
}

bool FileSystem_DeleteBitmapAndRaw(const char *name) {
    if (f_unlink(name) != FR_OK) {
        return false;
    }

    char belonging_raw[12] = {'\0'};

    uint32_t size = strlen(name);
    size -= 3;
    strncpy(belonging_raw, name, size);
    belonging_raw[size++] = 'b';
    belonging_raw[size++] = 'i';
    belonging_raw[size++] = 'n';

    if (f_unlink(belonging_raw) != FR_OK) {
        return false;
    }

    return rebuild_curr_bmp_window();
}

uint32_t FileSystem_AvailableFiles(void) {
    return available_bmps;
}
