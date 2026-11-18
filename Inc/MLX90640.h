#ifndef CAM_MLX90640_H
#define CAM_MLX90640_H

#include <stdbool.h>
#include <stdint.h>

int32_t MLX90640_Init(void);

int32_t MLX90640_Restart(void);

int32_t MLX90640_ReadTemperature(bool request_next);

int16_t *MLX90640_GetIRImageScaled(void);

#endif //CAM_MLX90640_H
