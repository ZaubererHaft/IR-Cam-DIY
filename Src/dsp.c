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
#define ALPHA_FX       96
#define ONE_MINUS_ALPHA (256 - ALPHA_FX)

#define SRC_WIDTH   32
#define SRC_HEIGHT  24

#define MAX_UPSCALING_FACTOR 8
#define MAX_DST_WIDTH        (SRC_WIDTH * MAX_UPSCALING_FACTOR)
#define MAX_NEW_PIXEL_SIZE   8

static int16_t old_data[SRC_WIDTH * SRC_HEIGHT];
static int16_t current_filtered[SRC_WIDTH * SRC_HEIGHT];

static uint16_t display_line_buffer[MAX_DST_WIDTH * MAX_NEW_PIXEL_SIZE];

void DSP_Init(void) {
    memset(old_data, 0xFF, sizeof(old_data));
}

static int16_t get_src_pixel(const int16_t *buffer, int32_t x, int32_t y) {
    if (x < 0) x = 0;
    if (x >= SRC_WIDTH) x = SRC_WIDTH - 1;
    if (y < 0) y = 0;
    if (y >= SRC_HEIGHT) y = SRC_HEIGHT - 1;
    return buffer[y * SRC_WIDTH + x];
}

int32_t upscaling_factor = 2;


// Der Upscaling-Faktor wird nun als Parameter übergeben
void DSP_ProcessAndDisplay() {
    // Dynamische Berechnung der Ziel-Dimensionen zur Laufzeit
    int32_t dst_width = SRC_WIDTH * upscaling_factor;
    int32_t dst_height = SRC_HEIGHT * upscaling_factor;
    int32_t new_pixel_size = 8 / upscaling_factor;

    if (new_pixel_size < 1) new_pixel_size = 1; // Sicherstellen, dass die Pixelgröße mindestens 1 ist

    const int16_t *image = MLX90640_GetIRImageScaled();

    // 1. EMA-Filter auf den 32x24 Rohdaten
    for (int i = 0; i < 768; i++) {
        int32_t temp_as_int = image[i];
        int32_t old_temp = old_data[i];
        current_filtered[i] = (int16_t) (((temp_as_int * ALPHA_FX) + (old_temp * ONE_MINUS_ALPHA)) >> 8);
        old_data[i] = current_filtered[i];
    }

    ILI9341_Set_Address(
        offset_x,
        offset_y,
        offset_x + (dst_width * new_pixel_size) - 1,
        offset_y + (dst_height * new_pixel_size) - 1
    );

    // 3. Bilineare Interpolation (Zeilenweise) mit variablem Faktor
    for (int32_t dst_y = 0; dst_y < dst_height; dst_y++) {
        int32_t src_y = dst_y / upscaling_factor;
        int32_t y_frac = dst_y % upscaling_factor;

        int32_t buf_idx = 0;

        for (int32_t dst_x = 0; dst_x < dst_width; dst_x++) {
            int32_t src_x = dst_x / upscaling_factor;
            int32_t x_frac = dst_x % upscaling_factor;

            // Die 4 umgebenden Quellpixel holen
            int32_t p00 = current_filtered[(src_y * SRC_WIDTH) + src_x];
            int32_t p10 = get_src_pixel(current_filtered, src_x + 1, src_y);
            int32_t p01 = get_src_pixel(current_filtered, src_x, src_y + 1);
            int32_t p11 = get_src_pixel(current_filtered, src_x + 1, src_y + 1);

            // Bilineare Interpolation Gewichtung
            int32_t w00 = (upscaling_factor - x_frac) * (upscaling_factor - y_frac);
            int32_t w10 = x_frac * (upscaling_factor - y_frac);
            int32_t w01 = (upscaling_factor - x_frac) * y_frac;
            int32_t w11 = x_frac * y_frac;

            // Division durch die dynamische Gesamtfläche (upscaling_factor^2)
            int32_t total_divisor = upscaling_factor * upscaling_factor;
            int16_t dst_p = (int16_t) ((p00 * w00 + p10 * w10 + p01 * w01 + p11 * w11) / total_divisor);

            uint16_t color_swapped = __REV16(Heatmap_TempScaledToRGB565(dst_p).raw);

            for (int p = 0; p < new_pixel_size; p++) {
                display_line_buffer[buf_idx++] = color_swapped;
            }
        }

        for (int r = 0; r < new_pixel_size; r++) {
            ILI9341_Send_Data_Block((uint8_t *) display_line_buffer, dst_width * new_pixel_size);
        }
    }
}

void DSP_ConfigObserver(Config config) {
}
