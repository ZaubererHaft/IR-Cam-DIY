#ifndef CAM_FILE_SYSTEM_H
#define CAM_FILE_SYSTEM_H

#include <stdint.h>
#include "config.h"

uint32_t FileSystem_Init(uint8_t *buffer, Config *out_config);

uint32_t FileSystem_WriteBitmap(const float *image, uint32_t size, const char *name);

uint32_t FileSystem_UpdateConfig(const Config *config);

#endif //CAM_FILE_SYSTEM_H