#include <string.h>

#include "ILI9341_DMA_driver.h"
#include "MLX90640_I2C_Driver.h"
#include "application.h"

#include "file_system.h"
#include "MLX90640.h"
#include "usbd_core.h"
#include "usb_device.h"
#include "usb_sync.h"
#include "user_interface.h"
#include "W25Qxx.h"

void Task_Init();

void Task_USB_Sync();

void Task_ReadIRData();

void Task_Draw();

static uint8_t sector_backup[4096] = {0};

USB_SYNC_Queue usb_synch_queue;
W25Q flash;

void application_main(void) {
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

  Task_Init();

  uint32_t time = HAL_GetTick();
  while (1) {
    Task_USB_Sync();
    // Task_ReadIRData();
    Task_Draw();

    if (HAL_GetTick() - time > 1000) {
      HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
      time = HAL_GetTick();
    }
  }
}


void Task_Init() {
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
  status |= FileSystem_Init(sector_backup);
  status |= FileSystem_WriteBitmap(MLX90640_GetIRImage(), 32 * 24);
  MX_USB_DEVICE_Init();
  //status |= MLX90640_Init();

  if (status != 0) {
    Error_Handler();
  }
}

void Task_USB_Sync() {
  int32_t status = 0;
  USB_SYNC *synch = USB_SYNC_Head(&usb_synch_queue);

  if (synch != NULL) {
    status |= W25Q_Write(&flash, synch->address, (uint8_t *) synch->USB_BlockBuffer,
                         sizeof(synch->USB_BlockBuffer), sector_backup);
    USB_SYNC_Head_DeallocateHead(&usb_synch_queue);
  }

  if (status != 0) {
    Error_Handler();
  }
}

void Task_ReadIRData() {
  if (!UserInterface_ShowMenu()) {
    //MLX90640_ReadAndDisplay();
  }
}

void Task_Draw() {
  UserInterface_Draw();
  UserInterface_RedrawIRImageIfNecessary(MLX90640_GetIRImage());
}
