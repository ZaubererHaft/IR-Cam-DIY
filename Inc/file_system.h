#ifndef CAM_FILE_SYSTEM_H
#define CAM_FILE_SYSTEM_H

#include <stdint.h>

int32_t FileSystem_Init();

int32_t FileSystem_WriteBitmap(float *image, uint32_t size);

#endif //CAM_FILE_SYSTEM_H