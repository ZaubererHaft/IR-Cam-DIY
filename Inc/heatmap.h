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
extern HeatmapFunction TempConverter;
extern HeatmapFunction available_heatmaps[];

extern float tMin;
extern float tMax;

extern uint32_t save_image;

#endif //CAM_HEATMAP_H