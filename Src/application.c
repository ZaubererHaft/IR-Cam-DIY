#include <string.h>

#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "ui_constants.h"
#include "heatmap.h"
#include "application.h"

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


static paramsMLX90640 mlxParams;

static uint16_t frame_1[834];
static uint16_t frame_2[834];

static uint16_t *data_frame = frame_2;
static uint16_t *display_frame = frame_1;
volatile int new_data_available = 0;
static uint16_t eeMLX90640[832];

static float image[32 * 24];
static float emissivity = 0.95f;
static int mlx_status;

volatile uint8_t SPI2_TX_completed_flag = 1; //flag indicating finish of SPI transmission

extern I2C_HandleTypeDef hi2c1;

void MLX90640_Init(void) {
  mlx_status |= MLX90640_SetRefreshRate(MLX90640_ADDR, RefreshRate);
  if (mlx_status != 0) {
    Error_Handler();
  }
  mlx_status |= MLX90640_SetChessMode(MLX90640_ADDR);
  if (mlx_status != 0) {
    Error_Handler();
  }
  mlx_status |= MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
  if (mlx_status != 0) {
    Error_Handler();
  }
  mlx_status |= MLX90640_ExtractParameters(eeMLX90640, &mlxParams);
  if (mlx_status != 0) {
    Error_Handler();
  }
}

int MLX90640_ReadAndDisplay(void) {
  if (new_data_available) {
    new_data_available = 0;

    MLX90640_CompleteFrameDataAsync(MLX90640_ADDR, data_frame);
    uint16_t *tmp = display_frame;
    display_frame = data_frame;
    data_frame = tmp;

    MLX90640_GetFrameDataAsync(MLX90640_ADDR, data_frame);
    float Ta = MLX90640_GetTa(display_frame, &mlxParams);
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    MLX90640_CalculateToAndDisplay(display_frame, &mlxParams, emissivity, tr, image, 0);

    return 1;
  }

  return 0;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c1) {
    new_data_available = 1;
  }
}

void redraw_ir_image() {
  for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
    int row = pixelNumber >> 5; // pixelNumber / 32
    int col = pixelNumber & 31; // pixelNumber % 32

    ILI9341_Draw_Rectangle(col * pixel_size + offset_x, row * pixel_size + offset_y, pixel_size, pixel_size,
                           TempConverter(image[pixelNumber]));
  }
}

static uint8_t sector_backup[4096] = {0};

USB_SYNC_Queue usb_synch_queue;

W25Q flash;

void application_main(void) {
  W25QInitParams params = {
    .page_size_byte = 256,
    .pages = 32768,
    .pages_per_sector = 16,
    .pages_per_blocks_small = 128,
    .pages_per_blocks_large = 256
  };


  W25Q_Init(&flash, &params);
  W25Q_Reset(&flash);


  W25Q_ChipErase(&flash);

  uint32_t ID;
  W25Q_ReadID(&flash, &ID);
  // char data1[] =  "Hello from W25Q1";
  // char data2[] =  "Hello from W25Q2";
  // char data3[] =  "Hello from W25Q3";

  // uint8_t rec[64] = {0};

  // //two pages, one sector
  // W25Q_Write(&flash, 250, (uint8_t *)data1, sizeof(data1), sector_backup);
  // memset(rec, 0, sizeof(rec));
  // W25Q_Read(&flash, 250, sizeof(data1), rec);

  // //two pages over two sectors
  // W25Q_Write(&flash, 4090, (uint8_t *)data2, sizeof(data2), sector_backup);
  // memset(rec, 0, sizeof(rec));
  // W25Q_Read(&flash, 250, sizeof(data1), rec);
  // memset(rec, 0, sizeof(rec));
  // W25Q_Read(&flash, 4090, sizeof(data2), rec);

  // //two pages over two sectors with starting sector != 0
  // W25Q_Write(&flash, 8190, (uint8_t *)data3, sizeof(data3), sector_backup);
  // memset(rec, 0, sizeof(rec));
  // W25Q_Read(&flash, 250, sizeof(data1), rec);
  // memset(rec, 0, sizeof(rec));
  // W25Q_Read(&flash, 4090, sizeof(data2), rec);
  // memset(rec, 0, sizeof(rec));
  // W25Q_Read(&flash, 8190, sizeof(data3), rec);

  UserInterface_Init();

  MX_USB_DEVICE_Init();

  while (1) {

    USB_SYNC *synch = USB_SYNC_Head(&usb_synch_queue);

    if (synch != NULL) {

      W25Q_Status status = W25Q_Write(&flash, synch->address, (uint8_t *) synch->USB_BlockBuffer,
                                      sizeof(synch->USB_BlockBuffer), sector_backup);
      if (status != W25Q_OK) {
        Error_Handler();
      } else {
        USB_SYNC_Head_DeallocateHead(&usb_synch_queue);
      }
    }
  }


  MLX90640_Init();

  int status = MLX90640_GetFrameDataAsync(MLX90640_ADDR, data_frame);
  if (status != 0) {
    Error_Handler();
  }

  while (1) {
    if (!UserInterface_ShowMenu()) {
      MLX90640_ReadAndDisplay();
    }

    UserInterface_Draw();

    if (UserInterface_NeedsIRImageRedraw()) {
      redraw_ir_image();
      UserInterface_IRImageRedrawn();
    }
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  SPI2_TX_completed_flag = 1;
}
