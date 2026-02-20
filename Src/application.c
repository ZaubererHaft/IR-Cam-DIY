#include <string.h>

#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "ui_constants.h"
#include "heatmap.h"
#include "application.h"

#include "ff_gen_drv.h"
#include "file_system.h"
#include "usbd_core.h"
#include "usbd_msc.h"
#include "usb_device.h"
#include "usb_sync.h"
#include "user_interface.h"
#include "W25Qxx.h"

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

static uint16_t ir_buffer_1[834];
static uint16_t ir_buffer_2[834];

static uint16_t *data_frame = ir_buffer_2;
static uint16_t *display_frame = ir_buffer_1;

static float image[32 * 24];
static float emissivity = 0.95f;
static int mlx_status;

static uint8_t sector_backup[4096] = {0};

volatile int new_ir_data_available = 0;
volatile uint8_t SPI2_TX_completed_flag = 1;

USB_SYNC_Queue usb_synch_queue;
W25Q flash;

int32_t MLX90640_Init(void) {
  uint16_t eeMLX90640[832];

  mlx_status |= MLX90640_SetRefreshRate(MLX90640_ADDR, RefreshRate);

  if (mlx_status == 0) {
    mlx_status |= MLX90640_SetChessMode(MLX90640_ADDR);
  }
  if (mlx_status == 0) {
    mlx_status |= MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
  }
  if (mlx_status == 0) {
    mlx_status |= MLX90640_ExtractParameters(eeMLX90640, &mlxParams);
  }

  return mlx_status;
}

int32_t MLX90640_ReadAndDisplay(void) {
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
    MLX90640_CalculateToAndDisplay(display_frame, &mlxParams, emissivity, tr, image, 0);
  }

  return status;
}


void redraw_ir_image() {
  for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
    int row = pixelNumber >> 5;
    int col = pixelNumber & 31;

    ILI9341_Draw_Rectangle(col * pixel_size + offset_x, row * pixel_size + offset_y, pixel_size, pixel_size,
                           TempConverter(image[pixelNumber]));
  }
}


int32_t Task_Init() {
  W25QInitParams params = {
    .page_size_byte = 256,
    .pages = 32768,
    .pages_per_sector = 16,
    .pages_per_blocks_small = 128,
    .pages_per_blocks_large = 256
  };

  int32_t status = W25Q_Init(&flash, &params);
  status |= W25Q_Reset(&flash);

  UserInterface_Init();
  status |= FileSystem_Init();
  status |= FileSystem_WriteBitmap(image, 32 * 24);
  MX_USB_DEVICE_Init();

  status |= MLX90640_Init();
  status |= MLX90640_GetFrameDataAsync(MLX90640_ADDR, data_frame);

  return status;
}

int32_t Task_USB_Sync() {
  int32_t status = 0;
  USB_SYNC *synch = USB_SYNC_Head(&usb_synch_queue);

  if (synch != NULL) {
    status |= W25Q_Write(&flash, synch->address, (uint8_t *) synch->USB_BlockBuffer,
                         sizeof(synch->USB_BlockBuffer), sector_backup);
  }

  return status;
}

int32_t Task_ReadIRData() {
  if (!UserInterface_ShowMenu()) {
    MLX90640_ReadAndDisplay();
  }

  return 0;
}

int32_t Task_Draw() {
  UserInterface_Draw();

  if (UserInterface_NeedsIRImageRedraw()) {
    redraw_ir_image();
    UserInterface_IRImageRedrawn();
  }

  return 0;
}

void application_main(void) {
  if (Task_Init() != 0) {
    Error_Handler();
  }

  while (1) {
    if (Task_USB_Sync() != 0) {
      Error_Handler();
    }
    if (Task_ReadIRData() != 0) {
      Error_Handler();
    }
    if (Task_Draw() != 0) {
      Error_Handler();
    }
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  SPI2_TX_completed_flag = 1;
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c1) {
    new_ir_data_available = 1;
  }
}
