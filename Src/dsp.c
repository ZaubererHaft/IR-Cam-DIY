#include "dsp.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "MLX90640.h"

#define offset_x  0
#define offset_y  24
#define pixel_size  4
#define ir_width  32
#define ir_height  24
#define lcd_width 320
#define lcd_height 240
#define menu_width 200
#define menu_height 80
#define menu_font_size 16

// Beispiel-Definitionen für den Filter (in den Header oder global legen):
// alpha_fx muss ein Wert zwischen 0 und 256 sein (256 = alpha von 1.0)
// Beispiel für alpha = 0.25: (0.25 * 256) = 64
#define ALPHA_FX       96
#define ONE_MINUS_ALPHA (256 - ALPHA_FX)

#define SRC_WIDTH   ir_width   // 32
#define SRC_HEIGHT  ir_height  // 24
#define DST_WIDTH   128        // Erneute Verdopplung (32 * 4)
#define DST_HEIGHT  96         // Erneute Verdopplung (24 * 4)
#define NEW_PIXEL_SIZE 2       // pixel_size halbiert auf 2, damit 128 * 2 = 256 Pixel Breite bleibt


static HeatmapFunction TemperatureConverter;
static uint16_t old_data[32 * 24];
// Der Puffer muss nun 128 * 2 = 256 Pixel breit sein
static uint16_t display_line_buffer[DST_WIDTH * NEW_PIXEL_SIZE];
static uint16_t current_filtered[SRC_WIDTH * SRC_HEIGHT];

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

    // 1. EMA-Filter auf den 32x24 Rohdaten
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

    // 3. 4x Bilineare Interpolation (Zeilenweise)
    // Wir laufen durch jede der 96 Zielzeilen
    for (int dst_y = 0; dst_y < DST_HEIGHT; dst_y++) {
        // Bestimme das Quellpixel und den fraktionalen Anteil (0, 1, 2 oder 3 Viertel)
        int src_y = dst_y >> 2;
        int y_frac = dst_y & 3;

        int buf_idx = 0;

        for (int dst_x = 0; dst_x < DST_WIDTH; dst_x++) {
            int src_x = dst_x >> 2;
            int x_frac = dst_x & 3;

            // Die 4 umgebenden Quellpixel holen
            uint32_t p00 = current_filtered[(src_y << 5) + src_x];
            uint32_t p10 = get_src_pixel(current_filtered, src_x + 1, src_y);
            uint32_t p01 = get_src_pixel(current_filtered, src_x, src_y + 1);
            uint32_t p11 = get_src_pixel(current_filtered, src_x + 1, src_y + 1);

            // Bilineare Interpolation über Gewichtung (Basis 4 * 4 = 16)
            // w00 = (4-x_frac)*(4-y_frac), w10 = x_frac*(4-y_frac), etc.
            uint32_t w00 = (4 - x_frac) * (4 - y_frac);
            uint32_t w10 = x_frac * (4 - y_frac);
            uint32_t w01 = (4 - x_frac) * y_frac;
            uint32_t w11 = x_frac * y_frac;

            // Gewichtete Summe bilden und durch 16 teilen (>> 4)
            uint16_t dst_p = (uint16_t) ((p00 * w00 + p10 * w10 + p01 * w01 + p11 * w11) >> 4);

            // In Farbe konvertieren und direkt in Big-Endian swappen
            uint16_t color_swapped = __REV16(TemperatureConverter((float) dst_p / TEMP_SCALING));

            // Entsprechend der neuen pixel_size (2) in den Zeilenpuffer schreiben
            for (int p = 0; p < NEW_PIXEL_SIZE; p++) {
                display_line_buffer[buf_idx++] = color_swapped;
            }
        }

        // Jede berechnete Interpolationszeile NEW_PIXEL_SIZE (2) mal per DMA senden
        for (int r = 0; r < NEW_PIXEL_SIZE; r++) {
            ILI9341_Send_Data_Block((uint8_t *) display_line_buffer, DST_WIDTH * NEW_PIXEL_SIZE);
        }
    }
}

void DSP_ConfigObserver(Config config) {
    TemperatureConverter = Heatmap_GetByIndex(config.heatmap_index);
}
