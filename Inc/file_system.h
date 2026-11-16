#ifndef CAM_FILE_SYSTEM_H
#define CAM_FILE_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#define FILESYSTEM_MAX_FILES 1000

typedef struct FileArray_ {
    char (*file_names)[12];
    uint32_t file_count;
} FileArray;

bool FileSystem_Init(uint8_t *tmp_buffer, Config *out_config);

bool FileSystem_WriteBitmap(const int16_t *image_scaled, uint32_t size, const char *name);

bool FileSystem_WriteRaw(const int16_t *image_raw, uint32_t size, const char *name);

bool FileSystem_WriteConfig(const Config *config);

bool FileSystem_FindBitmaps(FileArray *out_files);

bool FileSystem_DeleteBitmapAndRaw(const char *name);

uint32_t FileSystem_AvailableFiles(void);

#endif //CAM_FILE_SYSTEM_H
