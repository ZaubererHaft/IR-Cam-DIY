#ifndef CAM_HEATMAP_H
#define CAM_HEATMAP_H

#include <stdint.h>

#include "config.h"

typedef union {
    uint16_t raw;

    struct {
        uint16_t b: 5; // 0..31
        uint16_t g: 6; // 0..63
        uint16_t r: 5; // 0..31
    };
} RGB565_t;


RGB565_t Heatmap_TempScaledToRGB565(int16_t scaled_temp);

RGB565_t Heatmap_TempToRGB565(float temp_celsius);

const char *Heatmap_GetNameByIndex(uint32_t index);

uint32_t Heatmap_GetSize();

void Heatmap_ConfigObserver(Config config);

int16_t Temperature_FromFloat(float temp_cel);

float Temperature_ToFloat(int16_t temp_cel);


#endif //CAM_HEATMAP_H
