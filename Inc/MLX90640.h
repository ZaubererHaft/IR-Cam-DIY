#ifndef CAM_MLX90640_H
#define CAM_MLX90640_H

#include <stdint.h>

#include "config.h"


int32_t MLX90640_Init(void);

int32_t MLX90640_Restart(void);

int32_t MLX90640_ReadTemperature(uint32_t request_next);

uint16_t *MLX90640_GetIRImageScaled(void);

#endif //CAM_MLX90640_H
