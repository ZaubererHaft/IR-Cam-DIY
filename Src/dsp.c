#include "dsp.h"

#include <string.h>

#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "MLX90640.h"

#define offset_x  0
#define offset_y  24

#define SRC_WIDTH   32
#define SRC_HEIGHT  24

#define MAX_UPSCALING_FACTOR 8
#define MAX_DST_WIDTH        (SRC_WIDTH * MAX_UPSCALING_FACTOR)
#define MAX_NEW_PIXEL_SIZE   8

#define ALPHA_STILL       48
#define ALPHA_MOTION      220
#define MOTION_THRESHOLD  10

#define DIFF_MIN      7    // Bis 0,7°C Unterschied: reines Rauschen -> volle Glättung
#define DIFF_MAX      15   // Ab 1,5°C Unterschied: echte Bewegung -> kaum Glättung


static Config curr_config;

static int16_t old_processed_data[SRC_WIDTH * SRC_HEIGHT];
static int16_t curr_working_buffer[SRC_WIDTH * SRC_HEIGHT];
static int16_t next_working_buffer[SRC_WIDTH * SRC_HEIGHT];

static uint16_t display_line_buffer[MAX_DST_WIDTH * MAX_NEW_PIXEL_SIZE];

void DSP_Init(void) {
    memset(old_processed_data, 0xFF, sizeof(old_processed_data));
}

static void DSP_ApplyEMAFilter(int16_t *curr) {
    for (int i = 0; i < (SRC_WIDTH * SRC_HEIGHT); i++) {
        int32_t new_val = curr[i];
        int32_t old_val = old_processed_data[i];

        int32_t diff = new_val - old_val;
        if (diff < 0) {
            diff = -diff;
        }

        // select alpha value
        int32_t alpha;
        if (diff <= DIFF_MIN) {
            alpha = ALPHA_STILL;
        } else if (diff >= DIFF_MAX) {
            alpha = ALPHA_MOTION;
        } else {
            alpha = ALPHA_STILL + ((diff - DIFF_MIN) * (ALPHA_MOTION - ALPHA_STILL)) / (DIFF_MAX - DIFF_MIN);
        }

        int32_t one_minus_alpha = 256 - alpha;
        int16_t filtered = (int16_t) (((new_val * alpha) + (old_val * one_minus_alpha)) >> 8);

        curr[i] = filtered;
    }
}

// sorts 5 values and returns the median (at index 2)
static int16_t median_of_5(int16_t *window) {
    for (int i = 1; i < 5; i++) {
        int16_t key = window[i];
        int j = i - 1;
        while (j >= 0 && window[j] > key) {
            window[j + 1] = window[j];
            j--;
        }
        window[j + 1] = key;
    }
    return window[2];
}

static int16_t get_src_pixel(const int16_t *buffer, int32_t x, int32_t y) {
    if (x < 0) x = 0;
    if (x >= SRC_WIDTH) x = SRC_WIDTH - 1;
    if (y < 0) y = 0;
    if (y >= SRC_HEIGHT) y = SRC_HEIGHT - 1;
    return buffer[y * SRC_WIDTH + x];
}

static void DSP_ApplyMedianCross5(const int16_t *src, int16_t *dst) {
    int16_t window[5];

    for (int32_t y = 0; y < SRC_HEIGHT; y++) {
        for (int32_t x = 0; x < SRC_WIDTH; x++) {
            window[0] = get_src_pixel(src, x, y); // middle
            window[1] = get_src_pixel(src, x, y - 1); // top
            window[2] = get_src_pixel(src, x, y + 1); // bottom
            window[3] = get_src_pixel(src, x - 1, y); // left
            window[4] = get_src_pixel(src, x + 1, y); // right

            dst[y * SRC_WIDTH + x] = median_of_5(window);
        }
    }
}

static void DSP_ScaleUpAndDisplay(const int16_t *processed_data) {
    const int32_t upscaling_factor = (int32_t) curr_config.upscaling_factor;
    const int32_t dst_width = SRC_WIDTH * upscaling_factor;
    const int32_t dst_height = SRC_HEIGHT * upscaling_factor;
    const int32_t new_pixel_size = 8 / upscaling_factor;

    ILI9341_Set_Address(
        offset_x,
        offset_y,
        offset_x + (dst_width * new_pixel_size) - 1,
        offset_y + (dst_height * new_pixel_size) - 1
    );

    for (int32_t dst_y = 0; dst_y < dst_height; dst_y++) {
        int32_t src_y = dst_y / upscaling_factor;
        int32_t y_frac = dst_y % upscaling_factor;

        int32_t buf_idx = 0;

        for (int32_t dst_x = 0; dst_x < dst_width; dst_x++) {
            int32_t src_x = dst_x / upscaling_factor;
            int32_t x_frac = dst_x % upscaling_factor;

            // Die 4 umgebenden Quellpixel holen
            int32_t p00 = processed_data[(src_y * SRC_WIDTH) + src_x];
            int32_t p10 = get_src_pixel(processed_data, src_x + 1, src_y);
            int32_t p01 = get_src_pixel(processed_data, src_x, src_y + 1);
            int32_t p11 = get_src_pixel(processed_data, src_x + 1, src_y + 1);

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


void DSP_ProcessAndDisplay() {
    const int16_t *image = MLX90640_GetIRImageScaled();
    memcpy(curr_working_buffer, image, sizeof(curr_working_buffer));

    DSP_ApplyMedianCross5(curr_working_buffer, next_working_buffer);
    DSP_ApplyEMAFilter(next_working_buffer);

    const int16_t *processed_data = next_working_buffer;
    DSP_ScaleUpAndDisplay(processed_data);

    memcpy(old_processed_data, processed_data, sizeof(curr_working_buffer));
}


void DSP_ConfigObserver(Config config) {
    curr_config = config;
}
