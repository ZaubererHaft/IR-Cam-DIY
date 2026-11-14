#ifndef CAM_FILE_SYSTEM_H
#define CAM_FILE_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

bool FileSystem_Init(uint8_t *buffer, Config *out_config);

bool FileSystem_WriteBitmap(const int16_t *image_scaled, uint32_t size, const char *name);

bool FileSystem_WriteRaw(const int16_t *image_raw, uint32_t size, const char *name);

bool FileSystem_WriteConfig(const Config *config);

#endif //CAM_FILE_SYSTEM_H