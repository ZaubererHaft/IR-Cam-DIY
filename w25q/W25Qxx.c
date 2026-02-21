#include "W25Qxx.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

#define W25Q_SPI_DEFAULT_TIMEOUT_MS 2000

#define W25Q_NUMER_OF_SMALL_BLOCKS_UNSUPPORTED 512
#define W25_ID_SIZE 3

#define W25Q_CMD_ENABLE_RESET ((uint8_t)0x66)
#define W25Q_CMD_RESET ((uint8_t)0x99)
#define W25Q_CMD_READ_ID ((uint8_t)0x9F)
#define W25Q_ENABLE_READ ((uint8_t)0x03)
#define W25Q_ENABLE_FAST_READ ((uint8_t)0x0B)
#define W25Q_ENABLE_WRITE ((uint8_t)0x06)
#define W25Q_DISABLE_WRITE ((uint8_t)0x04)
#define W25Q_ERASE_SECTOR ((uint8_t)0x20)
#define W25Q_PAGE_PROGRAM ((uint8_t)0x02)
#define W25Q_CHIP_ERASE ((uint8_t)0x60)
#define W25Q_BUSY ((uint8_t)0x05)

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

W25Q_Status write_SPI(const uint8_t *data, uint16_t len);

W25Q_Status read_SPI(uint8_t *data, uint16_t len);

W25Q_Status write_enable(void);

W25Q_Status write_disable(void);

uint32_t bytes_to_write(const W25Q *flash, uint32_t size, uint16_t offset);

void cs_LOW(void);

void cs_HIGH(void);


W25Q_Status wait_for_ready(const W25Q *flash) {

    W25Q_Status status;
    uint32_t busy;
    do  {
        status = W25Q_Busy(flash, &busy);
    }
    while (busy && status == W25Q_OK);
    return status;
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

    // to simplify the driver, don't support larger flashes (would need 32 bit addresses)
    if (flash->blocks_small >= W25Q_NUMER_OF_SMALL_BLOCKS_UNSUPPORTED) {
        return W25Q_NOT_SUPPORTED;
    }
    return W25Q_OK;
}

W25Q_Status W25Q_Reset(const W25Q *flash) {
    UNUSED(flash);

    const uint8_t data[] = {W25Q_CMD_ENABLE_RESET, W25Q_CMD_RESET};

    cs_LOW();
    W25Q_Status status = write_SPI(data, sizeof(data));
    cs_HIGH();

    status |= wait_for_ready(flash);
    return status;
}

W25Q_Status W25Q_ReadJEDECIdentifier(const W25Q *flash, W25QJEDECIdentifier *out_id) {
    UNUSED(flash);

    W25Q_Status status = W25Q_OK;
    const uint8_t cmd = W25Q_CMD_READ_ID;

    cs_LOW();
    status |= write_SPI(&cmd, sizeof(cmd));
    status |= read_SPI((uint8_t *) &out_id->jedec_identifier, W25_ID_SIZE);
    cs_HIGH();

    if (status == W25Q_OK) {
        out_id->jedec_identifier = __REV(out_id->jedec_identifier) >> 8;
    }

    return status;
}

W25Q_Status do_read(const W25Q *flash, const uint32_t address_to_read, const uint32_t size, uint8_t *buffer,
                         const uint32_t mode) {
    W25Q_Status status = W25Q_OK;

    if (address_to_read + size > flash->flash_size_bytes) {
        status = W25Q_INVALID_ADDRESS;
    } else {
        const uint32_t data = __REV(address_to_read) | mode;
        cs_LOW();
        status |= write_SPI((const uint8_t *) &data, sizeof(data));
        status |= read_SPI(buffer, size);
        cs_HIGH();
    }

    return status;
}

W25Q_Status W25Q_FastRead(const W25Q *flash, const uint32_t address, const uint32_t size, uint8_t *buffer) {
    return do_read(flash, address, size, buffer, W25Q_ENABLE_FAST_READ);
}

W25Q_Status W25Q_Read(const W25Q *flash, const uint32_t address, const uint32_t size, uint8_t *buffer) {
    return do_read(flash, address, size, buffer, W25Q_ENABLE_READ);
}


W25Q_Status do_write_page(const W25Q *flash, const uint32_t page, const uint8_t *data) {
    W25Q_Status status = write_enable();
    const uint32_t cmd = __REV(page * flash->page_size_byte) | W25Q_PAGE_PROGRAM;

    cs_LOW();
    status |= write_SPI((const uint8_t *) &cmd, sizeof(cmd));
    status |= write_SPI(&data[0], flash->page_size_byte);
    cs_HIGH();

    status |= wait_for_ready(flash);
    return status;
}

W25Q_Status W25Q_Write(const W25Q *flash, const uint32_t address, const uint8_t *data, const uint32_t size,
                       uint8_t *sector_backup) {
    W25Q_Status status = W25Q_OK;

    const uint32_t sector_start = address / flash->sector_size_byte;
    const uint32_t sectors_to_delete = (address + size + flash->sector_size_byte - 1) / flash->sector_size_byte - sector_start;

    if (sector_start + sectors_to_delete > flash->sectors) {
        return W25Q_INVALID_ADDRESS;
    }

    uint32_t cur_address = address;
    uint32_t bytes_remaining = size;
    uint32_t data_index = 0;

    for (uint32_t i = 0; i < sectors_to_delete; i++) {
        const uint32_t cur_sector_address = (sector_start + i) * flash->sector_size_byte;

        // 1: create sector backup
        status |= W25Q_Read(flash, cur_sector_address, flash->sector_size_byte, sector_backup);

        // 2: delete full sector
        status |= W25Q_EraseSector(flash, sector_start + i);

        // 3: overwrite section backup with new data
        const uint32_t address_in_sector = cur_address - (sector_start + i) * flash->sector_size_byte;
        const uint32_t bytes_in_sector_remining = flash->sector_size_byte - address_in_sector;
        const uint32_t bytes_to_write_in_sector = MIN(bytes_in_sector_remining, bytes_remaining);

        for (uint32_t j = 0; j < bytes_to_write_in_sector; j++) {
            sector_backup[address_in_sector + j] = data[data_index + j];
        }

        // 4: write new section data back page-wise
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
    UNUSED(flash);

    W25Q_Status status = write_enable();

    cs_LOW();
    const uint8_t cmd = W25Q_CHIP_ERASE;
    status |= write_SPI(&cmd, sizeof(cmd));
    cs_HIGH();

    status |= wait_for_ready(flash);
    status |= write_disable();

    return status;
}

W25Q_Status W25Q_Busy(const W25Q *flash, uint32_t *is_busy) {
    UNUSED(flash);

    const uint8_t cmd = W25Q_BUSY;
    uint8_t busy;

    cs_LOW();
    W25Q_Status status = write_SPI(&cmd, sizeof(cmd));
    status |= read_SPI(&busy, sizeof(cmd));
    cs_HIGH();

    if (status == W25Q_OK) {
        *is_busy = (uint32_t) (busy & 0x01);
    }

    return status;
}

W25Q_Status W25Q_EraseSector(const W25Q *flash, const uint32_t sector) {
    W25Q_Status status = W25Q_OK;

    if (sector > flash->sectors) {
        status = W25Q_INVALID_SECTOR;
    } else {
        const uint32_t address_to_delete = sector * flash->sector_size_byte;
        const uint32_t cmd = __REV(address_to_delete) | W25Q_ERASE_SECTOR;

        status |= write_enable();
        cs_LOW();
        status |= write_SPI((const uint8_t *) &cmd, 4);
        cs_HIGH();

        status |= wait_for_ready(flash);
        status |= write_disable();
    }

    return status;
}

void cs_LOW(void) {
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

void cs_HIGH(void) {
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

W25Q_Status write_SPI(const uint8_t *data, const uint16_t len) {
    if (HAL_SPI_Transmit(&hspi3, data, len, W25Q_SPI_DEFAULT_TIMEOUT_MS) == HAL_OK) {
        return W25Q_OK;
    }

    return W25Q_SPI_COMM_ERROR;
}

W25Q_Status read_SPI(uint8_t *data, const uint16_t len) {
    if (HAL_SPI_Receive(&hspi3, data, len, W25Q_SPI_DEFAULT_TIMEOUT_MS) == HAL_OK) {
        return W25Q_OK;
    }
    return W25Q_SPI_COMM_ERROR;
}


/**
 *  Write Enable must be called before every page program
 */
W25Q_Status write_enable(void) {
    const uint8_t data = W25Q_ENABLE_WRITE;
    cs_LOW();
    const W25Q_Status status = write_SPI(&data, sizeof(data));
    cs_HIGH();

    return status;
}

/**
 *  Write Disable must be called before every page program
 */
W25Q_Status write_disable(void) {
    const uint8_t data = W25Q_DISABLE_WRITE;
    cs_LOW();
    const W25Q_Status status = write_SPI(&data, sizeof(data));
    cs_HIGH();

    return status;
}

uint32_t bytes_to_write(const W25Q *flash, const uint32_t size, const uint16_t offset) {
    if (size + offset < flash->page_size_byte) {
        return size;
    }

    return flash->page_size_byte - offset;
}
