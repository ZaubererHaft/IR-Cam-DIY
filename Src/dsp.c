#include "dsp.h"
#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "MLX90640.h"
#include "ui_constants.h"

#include <string.h>

#define MAX_UPSCALING_FACTOR 8
#define MAX_DST_WIDTH        (IR_SENSOR_WIDTH * MAX_UPSCALING_FACTOR)

#define ALPHA_STILL       48
#define ALPHA_MOTION      220
#define MOTION_THRESHOLD  10

#define DIFF_MIN      5    // Bis 0,5°C Unterschied: reines Rauschen -> volle Glättung
#define DIFF_MAX      15   // Ab 1,5°C Unterschied: echte Bewegung -> kaum Glättung

#define EDGE_THRESHOLD 10

static Config curr_config;

static int16_t old_processed_data[IR_SENSOR_WIDTH * IR_SENSOR_HEIGHT];
static int16_t curr_working_buffer[IR_SENSOR_WIDTH * IR_SENSOR_HEIGHT];
static int16_t next_working_buffer[IR_SENSOR_WIDTH * IR_SENSOR_HEIGHT];

static uint16_t display_line_buffer[MAX_DST_WIDTH];

void DSP_Init(void) {
    memset(old_processed_data, 20, sizeof(old_processed_data));
}

static void DSP_ApplyEMAFilter(int16_t *curr) {
    for (int i = 0; i < (IR_SENSOR_WIDTH * IR_SENSOR_HEIGHT); i++) {
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
    if (x >= IR_SENSOR_WIDTH) x = IR_SENSOR_WIDTH - 1;
    if (y < 0) y = 0;
    if (y >= IR_SENSOR_HEIGHT) y = IR_SENSOR_HEIGHT - 1;
    return buffer[y * IR_SENSOR_WIDTH + x];
}

static void DSP_ApplyMedianCross5(const int16_t *src, int16_t *dst) {
    int16_t window[5];

    for (int32_t y = 0; y < IR_SENSOR_HEIGHT; y++) {
        for (int32_t x = 0; x < IR_SENSOR_WIDTH; x++) {
            window[0] = get_src_pixel(src, x, y); // middle
            window[1] = get_src_pixel(src, x, y - 1); // top
            window[2] = get_src_pixel(src, x, y + 1); // bottom
            window[3] = get_src_pixel(src, x - 1, y); // left
            window[4] = get_src_pixel(src, x + 1, y); // right

            dst[y * IR_SENSOR_WIDTH + x] = median_of_5(window);
        }
    }
}

static int32_t abs32(int32_t v) {
    return (v < 0) ? -v : v;
}

static void DSP_ApplyEdgeAntiAliasing(const int16_t *src, int16_t *dst) {
    for (int32_t y = 0; y < IR_SENSOR_HEIGHT; y++) {
        for (int32_t x = 0; x < IR_SENSOR_WIDTH; x++) {
            int32_t c = get_src_pixel(src, x, y); // Mitte
            int32_t n = get_src_pixel(src, x, y - 1); // Oben
            int32_t s = get_src_pixel(src, x, y + 1); // Unten
            int32_t w = get_src_pixel(src, x - 1, y); // Links
            int32_t e = get_src_pixel(src, x + 1, y); // Rechts
            int32_t nw = get_src_pixel(src, x - 1, y - 1); // Oben-Links
            int32_t ne = get_src_pixel(src, x + 1, y - 1); // Oben-Rechts
            int32_t sw = get_src_pixel(src, x - 1, y + 1); // Unten-Links
            int32_t se = get_src_pixel(src, x + 1, y + 1); // Unten-Rechts

            // Kontrast (Gradient) entlang der 4 Hauptachsen berechnen
            int32_t grad_h = abs32(w - e); // Horizontal
            int32_t grad_v = abs32(n - s); // Vertikal
            int32_t grad_d1 = abs32(nw - se); // Diagonale "\"
            int32_t grad_d2 = abs32(ne - sw); // Diagonale "/"

            // Richtung mit dem GERINGSTEN Unterschied finden (= Verlauf der Kante)
            int32_t min_grad = grad_h;
            int32_t dir_avg = (w + e) >> 1;

            if (grad_v < min_grad) {
                min_grad = grad_v;
                dir_avg = (n + s) >> 1;
            }
            if (grad_d1 < min_grad) {
                min_grad = grad_d1;
                dir_avg = (nw + se) >> 1;
            }
            if (grad_d2 < min_grad) {
                min_grad = grad_d2;
                dir_avg = (ne + sw) >> 1;
            }

            // Nur glätten, wenn tatsächlich eine Kante vorliegt (Kontrast > Schwellwert),
            // aber entlang der Kante selbst ein homogenes Signal herrscht.
            int32_t max_grad = grad_h;
            if (grad_v > max_grad) max_grad = grad_v;
            if (grad_d1 > max_grad) max_grad = grad_d1;
            if (grad_d2 > max_grad) max_grad = grad_d2;

            // Schwellwert: z.B. 15 = 1,5°C Unterschied quer zur Kante
            if (max_grad > EDGE_THRESHOLD && min_grad < (max_grad >> 1)) {
                // Sanftes Blending (50% Original, 50% Kantenrichtung),
                // (c + dir_avg + 1) >> 1
                // // 75 % Original, 25 % Kantenrichtung
                dst[y * IR_SENSOR_WIDTH + x] = (int16_t) ((c * 3 + dir_avg + 2) >> 2);
            } else {
                // Flache Bereiche und feine Spitzen bleiben 100% scharf
                dst[y * IR_SENSOR_WIDTH + x] = (int16_t) c;
            }
        }
    }
}



static void DSP_ScaleUpAndDo(const int16_t *processed_data, ProcessedLineFunc func) {
    const int32_t upscaling_factor = (int32_t) curr_config.upscaling_factor;
    const int32_t dst_width = IR_SENSOR_WIDTH * upscaling_factor;
    const int32_t dst_height = IR_SENSOR_HEIGHT * upscaling_factor;
    const int32_t new_pixel_size = 8 / upscaling_factor;
    const int32_t line_pixel_count = dst_width * new_pixel_size;

    ILI9341_Set_Address(
        IR_IMAGE_BEGIN_X,
        IR_IMAGE_BEGIN_Y,
        IR_IMAGE_BEGIN_X + (dst_width * new_pixel_size) - 1,
        IR_IMAGE_BEGIN_Y + (dst_height * new_pixel_size) - 1
    );

    for (int32_t dst_y = 0; dst_y < dst_height; dst_y++) {
        int32_t src_y = dst_y / upscaling_factor;
        int32_t y_frac = dst_y % upscaling_factor;

        int32_t buf_idx = line_pixel_count - 1;

        for (int32_t dst_x = 0; dst_x < dst_width; dst_x++) {
            int32_t src_x = dst_x / upscaling_factor;
            int32_t x_frac = dst_x % upscaling_factor;

            int32_t p00 = processed_data[(src_y * IR_SENSOR_WIDTH) + src_x];
            int32_t p10 = get_src_pixel(processed_data, src_x + 1, src_y);
            int32_t p01 = get_src_pixel(processed_data, src_x, src_y + 1);
            int32_t p11 = get_src_pixel(processed_data, src_x + 1, src_y + 1);

            int32_t w00 = (upscaling_factor - x_frac) * (upscaling_factor - y_frac);
            int32_t w10 = x_frac * (upscaling_factor - y_frac);
            int32_t w01 = (upscaling_factor - x_frac) * y_frac;
            int32_t w11 = x_frac * y_frac;

            // Division by dynamic total area (upscaling_factor^2)
            int32_t total_divisor = upscaling_factor * upscaling_factor;
            int32_t weighted_sum = (p00 * w00) + (p10 * w10) + (p01 * w01) + (p11 * w11);
            int32_t half_div = total_divisor >> 1;

            if (weighted_sum >= 0) {
                weighted_sum += half_div;
            } else {
                weighted_sum -= half_div;
            }

            int16_t dst_p = (int16_t) (weighted_sum / total_divisor);
            uint16_t color_swapped = __REV16(Heatmap_TempScaledToRGB565(dst_p).raw);

            for (int p = 0; p < new_pixel_size; p++) {
                display_line_buffer[buf_idx--] = color_swapped;
            }
        }

        for (int r = 0; r < new_pixel_size; r++) {
            func(display_line_buffer, dst_width * new_pixel_size * 2);
        }
    }
}

static void send_to_dma(uint16_t *buff, uint32_t size) {
    ILI9341_Send_Data_Block((uint8_t *) buff, size);
}

void DSP_ProcessAndDo(ProcessedLineFunc func_after_process) {
    const int16_t *image = MLX90640_GetIRImageScaled();

    DSP_ApplyMedianCross5(image, next_working_buffer);
    DSP_ApplyEMAFilter(next_working_buffer);
    DSP_ApplyEdgeAntiAliasing(next_working_buffer, curr_working_buffer);
    DSP_ScaleUpAndDo(curr_working_buffer, func_after_process);
}


void DSP_ProcessAndDisplay() {
    DSP_ProcessAndDo(send_to_dma);
    memcpy(old_processed_data, curr_working_buffer, sizeof(curr_working_buffer));
}


void DSP_ConfigObserver(Config config) {
    curr_config = config;
}
