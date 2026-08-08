#ifndef CAM_HEATMAP_H
#define CAM_HEATMAP_H

#include <stdint.h>


uint16_t TempToRGB565(float temp);
uint16_t TempToGray565(float temp);
uint16_t TempToGray565_Inverted(float temp);
uint16_t TempToGray565_InvertedFast(float temp);
uint16_t TempToRainbow565_Fast(float temp);
uint16_t TempToMagma565_Fast(float temp);

typedef uint16_t (*HeatmapFunction)(float);

HeatmapFunction Heatmap_GetByIndex(uint32_t index);

extern float tMin;
extern float tMax;

#endif //CAM_HEATMAP_H