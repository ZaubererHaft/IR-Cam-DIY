#ifndef CAM_FILE_SYSTEM_H
#define CAM_FILE_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

bool FileSystem_Init(uint8_t *tmp_buffer, Config *out_config);

bool FileSystem_WriteBitmap(const int16_t *image_scaled, uint32_t size, const char *name);

bool FileSystem_WriteRaw(const int16_t *image_raw, uint32_t size, const char *name);

bool FileSystem_WriteConfig(const Config *config);

char *FileSystem_NextBitmap(void);

char *FileSystem_PreviousBitmap(void);

bool FileSystem_DeleteBitmapAndRaw(const char *name);

uint32_t FileSystem_AvailableFiles(void);

#endif //CAM_FILE_SYSTEM_H
