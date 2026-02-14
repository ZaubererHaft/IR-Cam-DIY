#ifndef CAM_W25QXX_H
#define CAM_W25QXX_H

#include <stdint.h>

void W25Q_Reset (void);

uint32_t W25Q_ReadID (void);

void W25Q_Write_Page (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);

void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);

#endif //CAM_W25QXX_H