#include "heatmap.h"

#define RGB565(r, g, b) \
(((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F))

uint16_t TempToRGB565(float temp) {
    // Clamp
    if (temp < tMin) temp = tMin;
    if (temp > tMax) temp = tMax;

    // Normalize (0.0 – 1.0)
    float norm = (temp - tMin) / (tMax - tMin);

    uint8_t r, g, b;

    // Blue → Green → Red
    if (norm < 0.5f) {
        // Blue → Green
        float t = norm * 2.0f;
        r = 0;
        g = (uint8_t) (t * 63);
        b = (uint8_t) ((1.0f - t) * 31);
    } else {
        // Green → Red
        float t = (norm - 0.5f) * 2.0f;
        r = (uint8_t) (t * 31);
        g = (uint8_t) ((1.0f - t) * 63);
        b = 0;
    }

    return RGB565(r, g, b);
}