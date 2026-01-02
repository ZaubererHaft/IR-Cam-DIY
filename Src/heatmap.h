#ifndef CAM_HEATMAP_H
#define CAM_HEATMAP_H

#include <stdint.h>

uint16_t TempToRGB565(float temp);

#define tMin 15.0f
#define tMax 37.0f

#endif //CAM_HEATMAP_H