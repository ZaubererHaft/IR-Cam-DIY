#include "ILI9341_DMA_driver.h"
#include "MLX90640_I2C_Driver.h"
#include "application.h"

#include "file_system.h"
#include "flash.h"
#include "heatmap.h"
#include "MLX90640.h"
#include "usb_device.h"
#include "user_interface.h"

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
static Config config;

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
  if (!UserInterface_Init()) {
    Error_Handler();
  }
  if (!Flash_Init()) {
    Error_Handler();
  }

  if (!FileSystem_Init(sector_backup, &config)) {
    Error_Handler();
  }

  if (MLX90640_Init() != MLX90640_NO_ERROR) {
    Error_Handler();
  }

  MX_USB_DEVICE_Init();
}


void Task_ReadIRData() {
  if (!UserInterface_ShowingMenu()) {
    MLX90640_ReadAndDisplay();
    menu_fresh_opened = 0;
  } else if (!menu_fresh_opened) {
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
    char buff[32] = {0};
    int32_t written = snprintf(buff, sizeof(buff), "img%lu.bmp", config.image_counter);
    if (written > 0) {
      uint32_t status = FileSystem_WriteBitmap(MLX90640_GetIRImage(), 32 * 24, buff);
      config.image_counter++;
      status |= FileSystem_UpdateConfig(&config);

      if (!status) {
        Error_Handler();
      }
    }
    save_image = 0;
  }
}
