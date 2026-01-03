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

uint16_t TempToGray565(float temp) {
    // Clamp
    if (temp < tMin) temp = tMin;
    if (temp > tMax) temp = tMax;

    // Normalize (0.0 – 1.0)
    float norm = (temp - tMin) / (tMax - tMin);

    // Convert to grayscale intensity
    uint8_t gray5 = (uint8_t)(norm * 31.0f); // for R and B (5-bit)
    uint8_t gray6 = (uint8_t)(norm * 63.0f); // for G (6-bit)

    return RGB565(gray5, gray6, gray5);
}

uint16_t TempToGray565_Inverted(float temp) {
    // Clamp
    if (temp < tMin) temp = tMin;
    if (temp > tMax) temp = tMax;

    // Normalize
    float norm = (temp - tMin) / (tMax - tMin);

    // Invert
    norm = 1.0f - norm;

    uint8_t gray5 = (uint8_t)(norm * 31.0f);
    uint8_t gray6 = (uint8_t)(norm * 63.0f);

    return RGB565(gray5, gray6, gray5);
}

uint16_t TempToGray565_InvertedFast(float temp) {
    if (temp < tMin) temp = tMin;
    if (temp > tMax) temp = tMax;

    float norm = (temp - tMin) / (tMax - tMin);

    // Invert
    norm = 1.0f - norm;

    // Fast gamma approx
    norm = norm * norm;

    uint8_t gray5 = (uint8_t)(norm * 31.0f);
    uint8_t gray6 = (uint8_t)(norm * 63.0f);

    return RGB565(gray5, gray6, gray5);
}

uint16_t TempToRainbow565_Fast(float temp) {
    if (temp < tMin) temp = tMin;
    if (temp > tMax) temp = tMax;

    uint8_t x = (uint8_t)(255.0f * (temp - tMin) / (tMax - tMin));

    uint8_t r = 0, g = 0, b = 0;

    if (x < 64) {
        r = 0;
        g = x;
        b = 31;
    } else if (x < 128) {
        r = 0;
        g = 63;
        b = 31 - ((x - 64) >> 1);
    } else if (x < 192) {
        r = (x - 128) >> 1;
        g = 63;
        b = 0;
    } else {
        r = 31;
        g = 63 - (x - 192);
        b = 0;
    }

    return RGB565(r, g, b);
}

uint16_t TempToMagma565_Fast(float temp) {
    if (temp < tMin) temp = tMin;
    if (temp > tMax) temp = tMax;

    uint8_t x = (uint8_t)(255.0f * (temp - tMin) / (tMax - tMin));

    uint8_t r = 0, g = 0, b = 0;

    if (x < 64) {
        r = x >> 3;
        g = 0;
        b = x >> 2;
    }
    else if (x < 128) {
        r = 8 + ((x - 64) >> 2);
        g = 0;
        b = 16 - ((x - 64) >> 3);
    }
    else if (x < 192) {
        r = 24 + ((x - 128) >> 4);
        g = (x - 128) >> 1;
        b = 0;
    }
    else {
        r = 31;
        g = 30 + ((x - 192) >> 2);
        b = (x - 192) >> 4;
        if (g > 63) g = 63;
    }

    return RGB565(r, g, b);
}