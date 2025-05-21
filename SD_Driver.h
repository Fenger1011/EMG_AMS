#ifndef SD_DRIVER_H
#define SD_DRIVER_H

#include <stdint.h>
#include "diskio.h"

void SPI_init(void);
uint8_t SPI_transmit(uint8_t data);
uint8_t SPI_receive(void);
void SPI_send_multi(const uint8_t* data, uint16_t len);
void SPI_receive_multi(uint8_t* data, uint16_t len);

uint8_t SD_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t SD_init(void);
uint8_t SD_writeSingleBlock(uint32_t block, const uint8_t* buff);
uint8_t SD_readSingleBlock(uint32_t block, uint8_t* buff);

DSTATUS disk_status(BYTE pdrv);
DSTATUS disk_initialize(BYTE pdrv);
DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count);
DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count);
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff);
DWORD get_fattime(void);

#endif
