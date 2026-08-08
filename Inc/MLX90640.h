#ifndef CAM_MLX90640_H
#define CAM_MLX90640_H

#include <stdint.h>

#include "config.h"


int32_t MLX90640_Init(void);

int32_t MLX90640_Restart(void);

int32_t MLX90640_ReadAndDisplay(uint32_t request_next);

float *MLX90640_GetIRImage(void);

void MLX90640_ConfigObserver(Config config);

#endif //CAM_MLX90640_H
