#ifndef CAM_HEATMAP_H
#define CAM_HEATMAP_H

#include <stdint.h>

#include "config.h"

typedef uint16_t (*HeatmapFunction)(float);

uint16_t Heatmap_TempScaledToRGB565(uint16_t scaled_temp);

uint16_t Heatmap_TempToRGB565(float temp);

const char * Heatmap_GetNameByIndex(uint32_t index);

uint32_t Heatmap_GetSize();

void Heatmap_ConfigObserver(Config config);

#define TEMP_SCALING 1000.0f

#endif //CAM_HEATMAP_H