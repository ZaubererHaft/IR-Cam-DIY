#include "flash.h"
#include "W25Qxx.h"
#include "main.h"

/**
 * Externally accessed
 */
W25Q flash;

extern SPI_HandleTypeDef hspi3;

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


uint32_t Flash_Init(void) {
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

    return status != W25Q_OK;
}
