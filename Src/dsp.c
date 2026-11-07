#include "dsp.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "MLX90640.h"
#include "ui_constants.h"

#define offset_x  0
#define offset_y  24
#define pixel_size  8
#define ir_width  32
#define ir_height  24
#define lcd_width 320
#define lcd_height 240
#define menu_width 200
#define menu_height 80
#define menu_font_size 16

//#define MIN_DIF_TO_REDRAW 0.3f //ToDo: make dynamic, e.g. (max - min) / 7
#define MIN_DIF_TO_REDRAW 30 //ToDo: make dynamic, e.g. (max - min) / 7

float min = 1000.0f; // Simplified FLT_MAX
float max = -1000.0f;

static HeatmapFunction TemperatureConverter;

static float alpha = 0.9f; // Wert zwischen 0 (starke Glättung) und 1 (keine Glättung)
int rescaled = 0;

static uint16_t old_data[32 * 24];


void DSP_Init(void) {
    memset(old_data, 0xFF, sizeof(old_data));
}


// Beispiel-Definitionen für den Filter (in den Header oder global legen):
// alpha_fx muss ein Wert zwischen 0 und 256 sein (256 = alpha von 1.0)
// Beispiel für alpha = 0.25: (0.25 * 256) = 64
#define ALPHA_FX       96
#define ONE_MINUS_ALPHA (256 - ALPHA_FX)

void DSP_ProcessAndDisplay(void) {
    const uint16_t *image = MLX90640_GetIRImageScaled();

    for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
        const uint32_t temp_as_int = image[pixelNumber];
        const uint32_t old_temp_as_int = old_data[pixelNumber];

        uint16_t filtered_temp = (uint16_t) (((temp_as_int * ALPHA_FX) + (old_temp_as_int * ONE_MINUS_ALPHA)) >> 8);

        // 2. Schwellenwert prüfen (0.5°C entsprechen bei Faktor 1000 genau dem Wert 50)
        if (abs((int) filtered_temp - (int) old_temp_as_int) >= 500) {
            const int row = pixelNumber >> 5; // pixelNumber / 32
            const int col = pixelNumber & 31; // pixelNumber % 32

            ILI9341_Draw_Rectangle(
                col * pixel_size + offset_x,
                row * pixel_size + offset_y,
                pixel_size,
                pixel_size,
                TemperatureConverter((float) filtered_temp / TEMP_SCALING)
            );

            old_data[pixelNumber] = filtered_temp;
        }
    }
}

void DSP_ConfigObserver(Config config) {
    TemperatureConverter = Heatmap_GetByIndex(config.heatmap_index);
}
