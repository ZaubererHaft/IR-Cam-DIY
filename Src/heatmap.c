#include "heatmap.h"

// Defines
#define TEMPERATURE_SCALING (10)
#define TEMPERATURE_MIN_VALUE (-20)
#define TEMPERATURE_MAX_VALUE (120)

#define LUT_SIZE ((TEMPERATURE_MAX_VALUE - TEMPERATURE_MIN_VALUE) * TEMPERATURE_SCALING)

#define GET_INDEX(scaled_temp) (scaled_temp - TEMPERATURE_MIN_VALUE * TEMPERATURE_SCALING)

// Typedefs
typedef RGB565_t (*HeatmapFunction)(int16_t);

// Prototypes
static RGB565_t ToMagma(int16_t temp);

static RGB565_t ToHotWhite(int16_t temp);

static RGB565_t ToGrayInverted(int16_t temp);

static RGB565_t ToGreen(int16_t temp);

static RGB565_t ToGray(int16_t temp);

static RGB565_t ToRed(int16_t temp);

// Static Variables
static const char *heatmap_names[] = {"Magma", "HotWhite", "Gray", "Gray (i)", "Red", "Green"};
static HeatmapFunction available_heatmaps[] = {
    &ToMagma, &ToHotWhite, &ToGray, &ToGrayInverted, &ToRed, &ToGreen
};
static RGB565_t color_lut[LUT_SIZE];
static uint16_t curr_temp_min;
static uint16_t curr_temp_max;

// Functions
RGB565_t Heatmap_TempScaledToRGB565(int16_t scaled_temp) {
    if (scaled_temp <= TEMPERATURE_MIN_VALUE * TEMPERATURE_SCALING) {
        return color_lut[0];
    }
    if (scaled_temp >= TEMPERATURE_MAX_VALUE * TEMPERATURE_SCALING) {
        return color_lut[LUT_SIZE - 1];
    }

    return color_lut[GET_INDEX(scaled_temp)];
}

RGB565_t Heatmap_TempToRGB565(float temp_celsius) {
    return Heatmap_TempScaledToRGB565(Temperature_FromFloat(temp_celsius));
}

const char *Heatmap_GetNameByIndex(uint32_t index) {
    if (index >= Heatmap_GetSize()) {
        return "Unknown";
    }
    return heatmap_names[index];
}

uint32_t Heatmap_GetSize() {
    return sizeof(available_heatmaps) / sizeof(available_heatmaps[0]);
}

static void init_lut(uint32_t heatmap_index) {
    const HeatmapFunction func = available_heatmaps[heatmap_index];

    for (int32_t i = 0; i < LUT_SIZE; i++) {
        int32_t scaled_temp = TEMPERATURE_MIN_VALUE * TEMPERATURE_SCALING + i;
        color_lut[i] = func((int16_t) scaled_temp);
    }
}

void Heatmap_ConfigObserver(Config config) {
    curr_temp_min = Temperature_FromFloat(config.tMin);
    curr_temp_max = Temperature_FromFloat(config.tMax);

    init_lut(config.heatmap_index);
}

int16_t Temperature_FromFloat(float temp_cel) {
    return (int16_t) (temp_cel * TEMPERATURE_SCALING);
}

float Temperature_ToFloat(int16_t temp_cel) {
    return (float) temp_cel / (float) TEMPERATURE_SCALING;
}

//Built-In Heatmaps
RGB565_t ToGreen(int16_t temp) {
    RGB565_t color = {.raw = 0};

    if (temp <= curr_temp_min) {
        color.g = 0;
        return color;
    }
    if (temp >= curr_temp_max) {
        color.g = 63;
        return color;
    }

    int32_t span = curr_temp_max - curr_temp_min;
    color.g = (uint16_t) (((int32_t) (temp - curr_temp_min) * 63) / span);

    return color;
}

RGB565_t ToRed(int16_t temp) {
    RGB565_t color = {.raw = 0};

    if (temp <= curr_temp_min) {
        color.r = 0;
        return color;
    }
    if (temp >= curr_temp_max) {
        color.r = 31;
        return color;
    }

    int32_t span = curr_temp_max - curr_temp_min;
    color.r = (uint16_t) (((int32_t) (temp - curr_temp_min) * 31) / span);

    return color;
}

static uint8_t NormalizeTemp255(int16_t temp) {
    if (temp <= curr_temp_min) return 0;
    if (temp >= curr_temp_max) return 255;

    int32_t span = curr_temp_max - curr_temp_min;
    return (uint8_t) (((int32_t) (temp - curr_temp_min) * 255) / span);
}

static RGB565_t MakeColor565(uint8_t r8, uint8_t g8, uint8_t b8) {
    RGB565_t color;
    color.r = r8 / 8; // 0..31
    color.g = g8 / 4; // 0..63
    color.b = b8 / 8; // 0..31
    return color;
}

RGB565_t ToGray(int16_t temp) {
    uint8_t val = NormalizeTemp255(temp);
    return MakeColor565(val, val, val);
}

RGB565_t ToGrayInverted(int16_t temp) {
    uint8_t val = 255 - NormalizeTemp255(temp);
    return MakeColor565(val, val, val);
}

RGB565_t ToHotWhite(int16_t temp) {
    uint8_t val = NormalizeTemp255(temp);
    uint8_t r, g, b;

    if (val < 85) {
        r = val * 3;
        g = 0;
        b = 0;
    } else if (val < 170) {
        r = 255;
        g = (val - 85) * 3;
        b = 0;
    } else {
        r = 255;
        g = 255;
        b = (val - 170) * 3;
    }
    return MakeColor565(r, g, b);
}

RGB565_t ToMagma(int16_t temp) {
    uint8_t val = NormalizeTemp255(temp);
    uint8_t r, g, b;

    if (val < 64) {
        r = val;
        g = 0;
        b = val * 2;
    } else if (val < 128) {
        uint8_t step = val - 64;
        r = 64 + (step * 2);
        g = step / 2;
        b = 128;
    } else if (val < 192) {
        uint8_t step = val - 128;
        r = 192 + step;
        g = 32 + (step * 2);
        b = 128 - (step * 2);
    } else {
        uint8_t step = val - 192;
        r = 255;
        g = 160 + ((step * 95) / 63);
        b = step * 3;
    }
    return MakeColor565(r, g, b);
}
