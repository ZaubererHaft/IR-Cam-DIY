#ifndef CAM_HEATMAP_H
#define CAM_HEATMAP_H

#include <stdint.h>

#include "config.h"

#define COLOR_RGB565 uint16_t

typedef COLOR_RGB565 (*HeatmapFunction)(float);

COLOR_RGB565 Heatmap_TempScaledToRGB565(int16_t scaled_temp);

COLOR_RGB565 Heatmap_TempToRGB565(float temp);

const char * Heatmap_GetNameByIndex(uint32_t index);

uint32_t Heatmap_GetSize();

void Heatmap_ConfigObserver(Config config);

int16_t Temperature_FromFloat(float temp_cel);

float Temperature_ToFloat(int16_t temp_cel);


#endif //CAM_HEATMAP_H