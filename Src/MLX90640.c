#include "MLX90640.h"

#include "heatmap.h"
#include "ILI9341_DMA_driver.h"
#include "MLX90640_API.h"
#include "main.h"
#include "ui_constants.h"

#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33
#define	 RefreshRate FPS16HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air

extern I2C_HandleTypeDef hi2c1;


static paramsMLX90640 mlxParams;
static float emissivity = 0.95f;

static uint16_t ir_buffer_1[834];
static uint16_t ir_buffer_2[834];
static uint16_t *data_frame = ir_buffer_2;
static uint16_t *display_frame = ir_buffer_1;

static float ir_image[32 * 24];

static int new_ir_data_available = 0;

int32_t MLX90640_Init(void) {
    uint16_t eeMLX90640[832];

    int32_t mlx_status = MLX90640_SetRefreshRate(MLX90640_ADDR, RefreshRate);

    if (mlx_status == MLX90640_NO_ERROR) {
        mlx_status |= MLX90640_SetChessMode(MLX90640_ADDR);
    }
    if (mlx_status == MLX90640_NO_ERROR) {
        mlx_status |= MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
    }
    if (mlx_status == MLX90640_NO_ERROR) {
        mlx_status |= MLX90640_ExtractParameters(eeMLX90640, &mlxParams);
    }
    if (mlx_status == MLX90640_NO_ERROR) {
        mlx_status |= MLX90640_GetFrameDataAsync(MLX90640_ADDR, data_frame);
    }
    return mlx_status;
}

int32_t do_read_and_display(int display) {
    int32_t status = 0;

    if (new_ir_data_available) {
        new_ir_data_available = 0;

        status = MLX90640_CompleteFrameDataAsync(MLX90640_ADDR, data_frame);
        uint16_t *tmp = display_frame;
        display_frame = data_frame;
        data_frame = tmp;

        status |= MLX90640_GetFrameDataAsync(MLX90640_ADDR, data_frame);
        float Ta = MLX90640_GetTa(display_frame, &mlxParams);
        float tr = Ta - TA_SHIFT;
        MLX90640_CalculateToAndDisplay(display_frame, &mlxParams, emissivity, tr, ir_image, 0, display);
    }


    return status;
}

int32_t MLX90640_Read(void) {
    return do_read_and_display(0);
}

int32_t MLX90640_ReadAndDisplay(void) {
    return do_read_and_display(1);
}


float *MLX90640_GetIRImage(void) {
    return ir_image;
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        new_ir_data_available = 1;
    }
}
