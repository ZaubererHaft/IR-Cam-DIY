#ifndef CAM_W25QXX_H
#define CAM_W25QXX_H

#include <stdint.h>

typedef enum W25Q_Status_ {
    W25Q_OK = 0x00000000,
    W25Q_NOT_SUPPORTED = 0x00000001,
    W25Q_SPI_COMM_ERROR = 0x00000002,
    W25Q_INVALID_ADDRESS = 0x00000004,
    W25Q_INVALID_SECTOR = 0x00000008,
} W25Q_Status;

typedef struct W25Q_ {
    uint32_t flash_size_bytes;
    uint32_t pages;
    uint32_t page_size_byte;
    uint32_t sectors;
    uint32_t sector_size_byte;
    uint32_t pages_per_sector;
    uint32_t blocks_small;
    uint32_t block_small_size_byte;
    uint32_t pages_per_blocks_small;
    uint32_t blocks_large;
    uint32_t block_large_size_byte;
    uint32_t pages_per_blocks_large;
} W25Q;

typedef struct W25QInitParams_ {
    uint32_t pages;
    uint32_t page_size_byte;
    uint32_t pages_per_sector;
    uint32_t pages_per_blocks_small;
    uint32_t pages_per_blocks_large;
} W25QInitParams;

W25Q_Status W25Q_Init(W25Q *flash, const W25QInitParams *params);

W25Q_Status W25Q_Reset(const W25Q *flash);

W25Q_Status W25Q_ReadID(const W25Q *flash, uint32_t *out_id);

W25Q_Status W25Q_Read(const W25Q *flash, uint32_t address, uint32_t size, uint8_t *buffer);

W25Q_Status W25Q_FastRead(const W25Q *flash, uint32_t address, uint32_t size, uint8_t *buffer);

W25Q_Status W25Q_EraseSector(const W25Q *flash, uint32_t sector);

W25Q_Status W25Q_Write(const W25Q *flash, uint32_t address, const uint8_t *data, uint32_t size,
                            uint8_t *sector_backup);

W25Q_Status W25Q_ChipErase(const W25Q *flash);

W25Q_Status W25Q_Busy(const W25Q *flash, uint32_t *is_busy);


#endif //CAM_W25QXX_H
