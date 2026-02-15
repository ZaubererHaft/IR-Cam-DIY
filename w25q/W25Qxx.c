#include "W25Qxx.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

#define numBLOCK 128

#define W25Q_CMD_ENABLE_RESET 0x66
#define W25Q_CMD_RESET 0x99
#define W25Q_CMD_READ_ID 0x9F
#define W25Q_ENABLE_READ 0x03
#define W25Q_ENABLE_FAST_READ 0x0B
#define W25Q_ENABLE_WRITE 0x06
#define W25Q_DISABLE_WRITE 0x04
#define W25Q_ERASE_SECTOR 0x20
#define W25Q_PAGE_PROGRAM 0x02

void csLOW(void) {
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
}

void csHIGH(void) {
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}

W25Q_Status SPI_Write(uint8_t *data, uint8_t len) {
    if (HAL_SPI_Transmit(&hspi3, data, len, 2000) == HAL_OK) {
        return W25Q_OK;
    }

    return W25Q_SPI_COMM_ERROR;
}

W25Q_Status SPI_Read(uint8_t *data, uint8_t len) {
    if (HAL_SPI_Receive(&hspi3, data, len, 5000) == HAL_OK) {
        return W25Q_OK;
    }
    return W25Q_SPI_COMM_ERROR;
}


/**
 *  Write Enable must be called before every page program
 */
W25Q_Status write_enable(void) {
    uint8_t tData = W25Q_ENABLE_WRITE;
    csLOW();
    W25Q_Status status = SPI_Write(&tData, 1);
    csHIGH();

    HAL_Delay(5);
    return status;
}

/**
 *  Write Disable must be called before every page program
 */
W25Q_Status write_disable(void) {
    uint8_t tData = W25Q_DISABLE_WRITE;
    csLOW();
    W25Q_Status status = SPI_Write(&tData, 1);
    csHIGH();

    HAL_Delay(5);
    return status;
}

uint32_t bytes_to_write(const W25Q *flash, uint32_t size, uint16_t offset) {
    if ((size + offset) < flash->page_size_byte) {
        return size;
    }

    return flash->page_size_byte - offset;
}


W25Q_Status W25Q_Write_Page(const W25Q *flash, uint32_t page, uint16_t offset, uint32_t size, uint8_t *data) {
    W25Q_Status status = W25Q_OK;

    uint32_t pages_to_write = (size + offset + flash->page_size_byte - 1) / flash->page_size_byte;
    uint16_t sector_start = page / flash->pages_per_sector;
    uint16_t sectors_to_delete = (pages_to_write + flash->pages_per_sector - 1) / flash->pages_per_sector;

    if (pages_to_write > flash->pages || sector_start + sectors_to_delete > flash->sectors) {
        status = W25Q_INVALID_ADDRESS;
    } else {
        for (uint16_t i = 0; i < sectors_to_delete; i++) {
            status |= W25Q_EraseSector(flash, sector_start + i);
        }

        uint32_t index = 0;

        for (uint32_t i = 0; i < pages_to_write; i++) {
            uint32_t address_to_write = ((page + i) * flash->page_size_byte) + offset;
            uint32_t bytes_to_send = bytes_to_write(flash, size, offset);

            status |= write_enable();
            uint32_t cmd = __REV(address_to_write) | W25Q_PAGE_PROGRAM;

            csLOW();
            status |= SPI_Write((uint8_t *) &cmd, 4);
            csHIGH();

            csLOW();
            status |= SPI_Write(&data[index], bytes_to_send);
            csHIGH();

            index += bytes_to_send;
            offset = 0;
            size -= bytes_to_send;

            HAL_Delay(5);
            status |= write_disable();
        }
    }

    return status;
}


W25Q_Status W25Q_EraseSector(const W25Q *flash, uint32_t sector) {
    W25Q_Status status = W25Q_OK;

    if (sector > flash->sectors) {
        status = W25Q_INVALID_SECTOR;
    } else {
        uint32_t address_to_delete = sector * flash->sector_size_byte;
        uint32_t cmd = __REV(address_to_delete) | W25Q_ERASE_SECTOR;

        status |= write_enable();
        csLOW();
        status |= SPI_Write((uint8_t *) &cmd, 4);
        csHIGH();
        HAL_Delay(450); // 450ms delay for sector erase
        status |= write_disable();
    }

    return status;
}

W25Q_Status W25Q_do_read(const W25Q *flash, uint32_t page, uint8_t page_offset_byte, uint32_t size, uint8_t *buffer,
                         uint32_t mode) {
    W25Q_Status status = W25Q_OK;

    const uint32_t address_to_read = (page * flash->page_size_byte) + page_offset_byte;

    if (address_to_read + size > flash->flash_size_bytes) {
        status = W25Q_INVALID_ADDRESS;
    } else {
        uint32_t tData = __REV(address_to_read) | mode;
        csLOW();
        status |= SPI_Write((uint8_t *) &tData, 4);
        status |= SPI_Read(buffer, size);
        csHIGH();
    }

    return status;
}

W25Q_Status W25Q_FastRead(const W25Q *flash, uint32_t page, uint8_t page_offset, uint32_t size, uint8_t *buffer) {
    return W25Q_do_read(flash, page, page_offset, size, buffer, W25Q_ENABLE_FAST_READ);
}

W25Q_Status W25Q_Read(const W25Q *flash, uint32_t page, uint8_t page_offset, uint32_t size, uint8_t *buffer) {
    return W25Q_do_read(flash, page, page_offset, size, buffer, W25Q_ENABLE_READ);
}

W25Q_Status W25Q_Init(W25Q *flash, const W25QInitParams *params) {
    flash->pages = params->pages;
    flash->page_size_byte = params->page_size_byte;
    flash->pages_per_sector = params->pages_per_sector;
    flash->pages_per_blocks_small = params->pages_per_blocks_small;
    flash->pages_per_blocks_large = params->pages_per_blocks_large;

    flash->flash_size_bytes = flash->page_size_byte * flash->pages;

    flash->sector_size_byte = flash->page_size_byte * flash->pages_per_sector;
    flash->sectors = flash->flash_size_bytes / flash->sector_size_byte;

    flash->block_small_size_byte = flash->pages_per_blocks_small * flash->sector_size_byte;
    flash->blocks_small = flash->flash_size_bytes / flash->block_small_size_byte;

    flash->block_large_size_byte = flash->pages_per_blocks_large * flash->sector_size_byte;
    flash->blocks_large = flash->flash_size_bytes / flash->block_large_size_byte;

    // to simplify the driver, don't support larger flashes (would need 32 addresses)
    if (flash->blocks_small >= 512) {
        return W25Q_NOT_SUPPORTED;
    }
    return W25Q_OK;
}

W25Q_Status W25Q_Reset(const W25Q *flash) {
    uint8_t tData[2] = {W25Q_CMD_ENABLE_RESET, W25Q_CMD_RESET};

    csLOW();
    const W25Q_Status status = SPI_Write(tData, 2);
    csHIGH();
    HAL_Delay(1);

    return status;
}

W25Q_Status W25Q_ReadID(const W25Q *flash, uint32_t *out_id) {
    W25Q_Status status = W25Q_OK;
    uint8_t cmd = W25Q_CMD_READ_ID;

    csLOW();
    status |= SPI_Write(&cmd, 1);
    status |= SPI_Read((uint8_t *) out_id, 3);
    csHIGH();

    if (status == W25Q_OK) {
        *out_id = __REV(*out_id) >> 8;
    }

    return status;
}
