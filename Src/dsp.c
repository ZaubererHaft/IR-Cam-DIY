#include "dsp.h"

#include <string.h>

#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "MLX90640.h"

#define offset_x  0
#define offset_y  24

// Beispiel-Definitionen für den Filter (in den Header oder global legen):
// alpha_fx muss ein Wert zwischen 0 und 256 sein (256 = alpha von 1.0)
// Beispiel für alpha = 0.25: (0.25 * 256) = 64
#define ALPHA_FX       128
#define ONE_MINUS_ALPHA (96 - ALPHA_FX)

// Flexibler Upscaling-Faktor (kann nun beliebig angepasst werden)
#define UPSCALING_FACTOR 4

#define SRC_WIDTH   32
#define SRC_HEIGHT  24
#define DST_WIDTH   (SRC_WIDTH * UPSCALING_FACTOR)
#define DST_HEIGHT  (SRC_HEIGHT * UPSCALING_FACTOR)
#define NEW_PIXEL_SIZE (8 / UPSCALING_FACTOR)


static HeatmapFunction TemperatureConverter;
static uint16_t old_data[SRC_WIDTH * SRC_HEIGHT];
static uint16_t current_filtered[SRC_WIDTH * SRC_HEIGHT];

static uint16_t display_line_buffer[DST_WIDTH * NEW_PIXEL_SIZE];

void DSP_Init(void) {
    memset(old_data, 0xFF, sizeof(old_data));
}


static uint32_t get_src_pixel(const uint16_t *buffer, int x, int y) {
    if (x < 0) x = 0;
    if (x >= SRC_WIDTH) x = SRC_WIDTH - 1;
    if (y < 0) y = 0;
    if (y >= SRC_HEIGHT) y = SRC_HEIGHT - 1;
    return buffer[y * SRC_WIDTH + x];
}

void DSP_ProcessAndDisplay(void) {
    const uint16_t *image = MLX90640_GetIRImageScaled();

    // Filter data (here recursive Pole for noise elimination)
    for (int i = 0; i < 768; i++) {
        uint32_t temp_as_int = image[i];
        uint32_t old_temp = old_data[i];
        current_filtered[i] = (uint16_t) (((temp_as_int * ALPHA_FX) + (old_temp * ONE_MINUS_ALPHA)) >> 8);
        old_data[i] = current_filtered[i];
    }

    ILI9341_Set_Address(
        offset_x,
        offset_y,
        offset_x + (DST_WIDTH * NEW_PIXEL_SIZE) - 1,
        offset_y + (DST_HEIGHT * NEW_PIXEL_SIZE) - 1
    );

    // Bilinear interpolation
    for (int dst_y = 0; dst_y < DST_HEIGHT; dst_y++) {
        // Bestimme das Quellpixel und den fraktionalen Anteil basierend auf dem Faktor
        int src_y = dst_y / UPSCALING_FACTOR;
        int y_frac = dst_y % UPSCALING_FACTOR;

        int buf_idx = 0;

        for (int dst_x = 0; dst_x < DST_WIDTH; dst_x++) {
            int src_x = dst_x / UPSCALING_FACTOR;
            int x_frac = dst_x % UPSCALING_FACTOR;

            // Die 4 umgebenden Quellpixel holen
            uint32_t p00 = current_filtered[(src_y * SRC_WIDTH) + src_x];
            uint32_t p10 = get_src_pixel(current_filtered, src_x + 1, src_y);
            uint32_t p01 = get_src_pixel(current_filtered, src_x, src_y + 1);
            uint32_t p11 = get_src_pixel(current_filtered, src_x + 1, src_y + 1);

            // Bilineare Interpolation über Gewichtung (Basis: UPSCALING_FACTOR * UPSCALING_FACTOR)
            uint32_t w00 = (UPSCALING_FACTOR - x_frac) * (UPSCALING_FACTOR - y_frac);
            uint32_t w10 = x_frac * (UPSCALING_FACTOR - y_frac);
            uint32_t w01 = (UPSCALING_FACTOR - x_frac) * y_frac;
            uint32_t w11 = x_frac * y_frac;

            // Gewichtete Summe bilden und durch die Gesamtfläche teilen
            uint32_t total_divisor = UPSCALING_FACTOR * UPSCALING_FACTOR;
            uint16_t dst_p = (uint16_t) ((p00 * w00 + p10 * w10 + p01 * w01 + p11 * w11) / total_divisor);

            uint16_t color_swapped = __REV16(TemperatureConverter((float) dst_p / TEMP_SCALING));

            for (int p = 0; p < NEW_PIXEL_SIZE; p++) {
                display_line_buffer[buf_idx++] = color_swapped;
            }
        }

        for (int r = 0; r < NEW_PIXEL_SIZE; r++) {
            ILI9341_Send_Data_Block((uint8_t *) display_line_buffer, DST_WIDTH * NEW_PIXEL_SIZE);
        }
    }
}

void DSP_ConfigObserver(Config config) {
    TemperatureConverter = Heatmap_GetByIndex(config.heatmap_index);
}
