#ifndef CAM_MLX90640_H
#define CAM_MLX90640_H

#include <stdint.h>

int32_t MLX90640_Init(void);

int32_t MLX90640_Complete(void);

int32_t MLX90640_ReadAndDisplay(void);

float *MLX90640_GetIRImage(void);

#endif //CAM_MLX90640_H
