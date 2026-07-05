#include "ILI9341_DMA_driver.h"
#include "MLX90640_I2C_Driver.h"
#include "application.h"

#include "file_system.h"
#include "heatmap.h"
#include "MLX90640.h"
#include "usb_device.h"
#include "user_interface.h"
#include "W25Qxx.h"

extern SPI_HandleTypeDef hspi3;

void Task_Init();

void Task_ReadIRData();

void Task_Draw();

static uint8_t sector_backup[4096] = {0};

W25Q flash;

void application_main(void) {
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
  Task_Init();

  uint32_t time = HAL_GetTick();
  while (1) {
    // Task_ReadIRData();
    Task_Draw();

    if (save_image) {
      (void) FileSystem_WriteBitmap(MLX90640_GetIRImage(), 32 * 24, "test3.bmp");
      save_image = 0;
    }

    if (HAL_GetTick() - time > 1000) {
      HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
      time = HAL_GetTick();
    }
  }
}

void W25Q_GPIO(uint32_t value) {
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, value);
}


W25Q_Status W25Q_SPI_Write(uint8_t *data, const uint16_t len) {
  if (HAL_SPI_Transmit(&hspi3, data, len, 500) == HAL_OK) {
    return W25Q_OK;
  }

  return W25Q_SPI_COMM_ERROR;
}

W25Q_Status W25Q_SPI_Read(uint8_t *data, const uint16_t len) {
  if (HAL_SPI_Receive(&hspi3, data, len, 500) == HAL_OK) {
    return W25Q_OK;
  }
  return W25Q_SPI_COMM_ERROR;
}


void Task_Init() {
  const W25QInitParams params = {
    .page_size_byte = 256,
    .pages = 32768 * 2,
    .pages_per_sector = 16,
    .pages_per_blocks_small = 128,
    .pages_per_blocks_large = 256,
    .gpio_function = W25Q_GPIO,
    .spi_read = W25Q_SPI_Read,
    .spi_write = W25Q_SPI_Write
  };

  W25Q_Status status = W25Q_Init(&flash, &params);
  status |= W25Q_Reset(&flash);

  UserInterface_Init();
  status |= FileSystem_Init(sector_backup);

  //status |= MLX90640_Init();
  MX_USB_DEVICE_Init();

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
