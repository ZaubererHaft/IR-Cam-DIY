#ifndef CAM_FILE_SYSTEM_H
#define CAM_FILE_SYSTEM_H

#include <stdint.h>

int32_t FileSystem_Init(uint8_t *buffer);

int32_t FileSystem_WriteBitmap(float *image, uint32_t size, const char *name);

#endif //CAM_FILE_SYSTEM_H