#include "ILI9341_DMA_driver.h"
#include "MLX90640_I2C_Driver.h"
#include "application.h"

#include "file_system.h"
#include "flash.h"
#include "heatmap.h"
#include "MLX90640.h"
#include "usb_device.h"
#include "user_interface.h"
#include "analog.h"

/**
 * Prototypes
 */
void Task_Init(void);

void Task_ReadIRData(void);

void Task_ReadAnalogData(void);

void Task_Draw(void);

void Task_WriteImage(void);

void Task_SynchConfig(void);

void Task_BlinkyDog(void);

/**
 * Local variables
 */
static uint8_t sector_backup[4096] = {0};
static int32_t menu_fresh_opened = 0;
static Config config;
static uint32_t time = 0;
static uint32_t time_analog = 0;
static uint32_t synch_config = 0;
static uint16_t adc_data[3] = {0};

void application_main(void) {
  Task_Init();

  while (1) {
    Task_ReadIRData();
    Task_ReadAnalogData();
    Task_Draw();
    Task_WriteImage();
    Task_SynchConfig();
    Task_BlinkyDog();
  }
}


void Task_Init(void) {
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

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

  if (MLX90640_Restart() != MLX90640_NO_ERROR) {
    Error_Handler();
  }

  time = HAL_GetTick();
}


void Task_ReadIRData(void) {
  if (UserInterface_ShowingMenu()) {
    menu_fresh_opened = 1;
    MLX90640_ReadAndDisplay(0);
  } else {
    if (menu_fresh_opened) {
      // menu closed -> restart
      MLX90640_Restart();
    } else {
      MLX90640_ReadAndDisplay(1);
    }
    menu_fresh_opened = 0;
  }
}

void Task_ReadAnalogData(void) {

  if (UserInterface_ShowingMenu() || HAL_GetTick() - time_analog > 5000) {
    if (!Analog_PollADCData(adc_data)) {
      Error_Handler();
    }
    time_analog = HAL_GetTick();
  }

  UserInterface_PutAnalogData(adc_data);
}


void Task_Draw(void) {
  UserInterface_Draw();
  UserInterface_RedrawIRImageIfNecessary(MLX90640_GetIRImage());
}


void Task_WriteImage(void) {
  if (save_image) {
    char buff[32] = {0};
    int32_t written = snprintf(buff, sizeof(buff), "img%lu.bmp", config.image_counter);
    if (written > 0) {
      uint32_t status = FileSystem_WriteBitmap(MLX90640_GetIRImage(), 32 * 24, buff);
      config.image_counter++;
      synch_config = 1;

      if (!status) {
        Error_Handler();
      }
    }
    save_image = 0;
  }
}

void Task_SynchConfig(void) {
  if (synch_config) {
    if (!FileSystem_UpdateConfig(&config)) {
      Error_Handler();
    }

    synch_config = 0;
  }
}


void Task_BlinkyDog(void) {
  if (HAL_GetTick() - time > 1000) {
    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    time = HAL_GetTick();
  }
}
