#include "W25Qxx.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

#define W25Q_CMD_ENABLE_RESET 0x66
#define W25Q_CMD_RESET 0x99
#define W25Q_CMD_READ_ID 0x9F
#define W25Q_ENABLE_READ 0x03
#define W25Q_ENABLE_FAST_READ 0x0B
#define W25Q_ENABLE_WRITE 0x06
#define W25Q_DISABLE_WRITE 0x04
#define W25Q_ERASE_SECTOR 0x20
#define W25Q_PAGE_PROGRAM 0x02
#define W25Q_CHIP_ERASE 0x60
#define W25Q_BUSY 0x05

W25Q_Status SPI_Write(uint8_t *data, uint16_t len);

W25Q_Status SPI_Read(uint8_t *data, uint16_t len);

W25Q_Status write_enable(void);

W25Q_Status write_disable(void);

uint32_t bytes_to_write(const W25Q *flash, uint32_t size, uint16_t offset);

void csLOW(void);

void csHIGH(void);


#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

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

W25Q_Status W25Q_do_read(const W25Q *flash, uint32_t address_to_read, uint32_t size, uint8_t *buffer,
                         uint32_t mode) {
    W25Q_Status status = W25Q_OK;

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

W25Q_Status W25Q_FastRead(const W25Q *flash, uint32_t address, uint32_t size, uint8_t *buffer) {
    return W25Q_do_read(flash, address, size, buffer, W25Q_ENABLE_FAST_READ);
}

W25Q_Status W25Q_Read(const W25Q *flash, uint32_t address, uint32_t size, uint8_t *buffer) {
    return W25Q_do_read(flash, address, size, buffer, W25Q_ENABLE_READ);
}


W25Q_Status do_write_page(const W25Q *flash, uint32_t page, uint8_t *data) {
    W25Q_Status status = write_enable();
    uint32_t cmd = __REV(page * flash->page_size_byte) | W25Q_PAGE_PROGRAM;

    csLOW();
    status |= SPI_Write((uint8_t *) &cmd, 4);
    status |= SPI_Write(&data[0], flash->page_size_byte);
    csHIGH();

    HAL_Delay(5);

    return status;
}

W25Q_Status W25Q_Write(const W25Q *flash, const uint32_t address, const uint8_t *data, const uint32_t size,
                       uint8_t *sector_backup) {
    W25Q_Status status = W25Q_OK;

    const uint32_t sector_start = address / flash->sector_size_byte;
    const uint32_t sectors_to_delete = (address + size + flash->sector_size_byte - 1) / flash->sector_size_byte - sector_start;

    uint32_t cur_address = address;
    uint32_t bytes_remaining = size;
    uint32_t data_index = 0;

    for (uint32_t i = 0; i < sectors_to_delete; i++) {
        const uint32_t cur_sector_address = (sector_start + i) * flash->sector_size_byte;

        // create sector backup
        status |= W25Q_Read(flash, cur_sector_address, flash->sector_size_byte, sector_backup);

        // delete full sector
        status |= W25Q_EraseSector(flash, sector_start + i);

        // overwrite section backup with new data
        const uint32_t address_in_sector = cur_address - (sector_start + i) * flash->sector_size_byte;
        const uint32_t bytes_in_sector_remining = flash->sector_size_byte - address_in_sector;
        const uint32_t bytes_to_write_in_sector = MIN(bytes_in_sector_remining, bytes_remaining);

        for (uint32_t j = 0; j < bytes_to_write_in_sector; j++) {
            sector_backup[address_in_sector + j] = data[data_index + j];
        }

        // write new section data back page-wise
        for (uint32_t j = 0; j < flash->pages_per_sector; j++) {
            //"page" is the absolute page in the flash, whereas "memory" is the local index in the section backup
            const uint32_t page = (sector_start + i) * flash->pages_per_sector + j;
            const uint32_t memory = j * flash->page_size_byte;
            status |= do_write_page(flash, page, &sector_backup[memory]);
        }

        data_index += bytes_to_write_in_sector;
        bytes_remaining -= bytes_to_write_in_sector;
        cur_address = cur_sector_address + flash->sector_size_byte;
    }

    return status;
}

W25Q_Status W25Q_ChipErase(const W25Q *flash) {
    W25Q_Status status = write_enable();
    uint32_t cmd = W25Q_CHIP_ERASE;

    csLOW();
    status |= SPI_Write((uint8_t *) &cmd, 1);
    csHIGH();

    HAL_Delay(20 * 1000);

    return status;
}

W25Q_Status W25Q_Busy(const W25Q *flash, uint32_t *is_busy) {
    W25Q_Status status = write_enable();
    uint32_t cmd = W25Q_BUSY;

    uint8_t busy;

    csLOW();
    status |= SPI_Write((uint8_t *) &cmd, 4);
    status |= SPI_Read(&busy, 1);
    csHIGH();

    if (status == W25Q_OK) {
        *is_busy = (uint32_t) (busy & 0x01);
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

void csLOW(void) {
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
}

void csHIGH(void) {
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}

W25Q_Status SPI_Write(uint8_t *data, uint16_t len) {
    if (HAL_SPI_Transmit(&hspi3, data, len, 2000) == HAL_OK) {
        return W25Q_OK;
    }

    return W25Q_SPI_COMM_ERROR;
}

W25Q_Status SPI_Read(uint8_t *data, uint16_t len) {
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
