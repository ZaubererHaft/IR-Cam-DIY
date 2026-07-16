#include "ILI9341_DMA_driver.h"
#include "MLX90640_I2C_Driver.h"
#include "application.h"

#include "file_system.h"
#include "flash.h"
#include "heatmap.h"
#include "MLX90640.h"
#include "usb_device.h"
#include "user_interface.h"
#include "W25Qxx.h"

/**
 * Prototypes
 */
void Task_Init();
void Task_ReadIRData();
void Task_Draw();
void Task_WriteImage();



/**
 * Local variables
 */
static uint8_t sector_backup[4096] = {0};
static int32_t menu_fresh_opened = 0;


void application_main(void) {
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
  Task_Init();

  uint32_t time = HAL_GetTick();
  while (1) {
    Task_ReadIRData();
    Task_Draw();
    Task_WriteImage();

    if (HAL_GetTick() - time > 1000) {
      HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
      time = HAL_GetTick();
    }
  }
}



void Task_Init() {
  uint32_t status = 0;

  UserInterface_Init();

  status = Flash_Init();
  status |= FileSystem_Init(sector_backup);
  MX_USB_DEVICE_Init();
  status |= MLX90640_Init();

  if (status != 0) {
    Error_Handler();
  }
}



void Task_ReadIRData() {
  if (!UserInterface_ShowingMenu()) {
    MLX90640_ReadAndDisplay();
    menu_fresh_opened = 0;
  }
  else if (!menu_fresh_opened) {
    MLX90640_Read();
    menu_fresh_opened = 1;
  }
}


void Task_Draw() {
  UserInterface_Draw();
  UserInterface_RedrawIRImageIfNecessary(MLX90640_GetIRImage());
}


void Task_WriteImage() {
  if (save_image) {
    (void) FileSystem_WriteBitmap(MLX90640_GetIRImage(), 32 * 24, "test3.bmp");
    save_image = 0;
  }
}