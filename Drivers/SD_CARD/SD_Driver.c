#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "diskio.h"
#include "ff.h"
#include "SD_Driver.h"

// --- Adjusted for Mega 2560 SPI pins (PORTB) ---
#define SPI_DDR    DDRB
#define SPI_PORT   PORTB
#define DD_MOSI    DDB2   // Pin 51
#define DD_MISO    DDB3   // Pin 50
#define DD_SCK     DDB1   // Pin 52
#define DD_SS      DDB0   // Pin 53

#define CS_HIGH()  (SPI_PORT |= (1 << DD_SS))
#define CS_LOW()   (SPI_PORT &= ~(1 << DD_SS))

#define CMD0    0
#define CMD8    8
#define CMD17   17
#define CMD24   24
#define CMD55   55
#define CMD58   58
#define ACMD41  41

#define DEV_MMC 0

static volatile DSTATUS Stat = STA_NOINIT;

void SPI_init(void) {
	SPI_DDR |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS);
	SPI_DDR &= ~(1<<DD_MISO);
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0); // slow
	SPSR = 0;
	CS_HIGH();
}

uint8_t SPI_transmit(uint8_t data) {
	SPDR = data;
	while (!(SPSR & (1<<SPIF)));
	return SPDR;
}

uint8_t SPI_receive(void) {
	return SPI_transmit(0xFF);
}

void SPI_send_multi(const uint8_t* data, uint16_t len) {
	while (len--) SPI_transmit(*data++);
}

void SPI_receive_multi(uint8_t* data, uint16_t len) {
	while (len--) *data++ = SPI_receive();
}

uint8_t SD_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
	uint8_t response, retry = 0;
	CS_LOW();
	SPI_transmit(0xFF);
	SPI_transmit(0x40 | cmd);
	SPI_transmit((uint8_t)(arg >> 24));
	SPI_transmit((uint8_t)(arg >> 16));
	SPI_transmit((uint8_t)(arg >> 8));
	SPI_transmit((uint8_t)arg);
	SPI_transmit(crc);
	do {
		response = SPI_receive();
	} while ((response & 0x80) && ++retry < 10);
	return response;
}

uint8_t SD_init(void) {
	uint8_t i, response;
	SPI_init();
	CS_HIGH();
	for (i = 0; i < 10; i++) SPI_transmit(0xFF);
	for (i = 0; i < 10; i++) {
		response = SD_send_cmd(CMD0, 0, 0x95);
		CS_HIGH(); SPI_transmit(0xFF);
		if (response == 0x01) break;
		_delay_ms(10);
	}
	if (response != 0x01) return 1;
	response = SD_send_cmd(CMD8, 0x1AA, 0x87);
	for (i = 0; i < 4; i++) SPI_receive();
	CS_HIGH(); SPI_transmit(0xFF);
	if (response != 0x01) return 2;
	for (i = 0; i < 100; i++) {
		SD_send_cmd(CMD55, 0, 0x65);
		response = SD_send_cmd(ACMD41, 0x40000000, 0x77);
		CS_HIGH(); SPI_transmit(0xFF);
		if (response == 0x00) break;
		_delay_ms(10);
	}
	if (response != 0x00) return 3;
	response = SD_send_cmd(CMD58, 0, 0);
	for (i = 0; i < 4; i++) SPI_receive();
	CS_HIGH(); SPI_transmit(0xFF);
	SPCR = (1<<SPE)|(1<<MSTR);
	return 0;
}

uint8_t SD_writeSingleBlock(uint32_t block, const uint8_t* buff) {
	uint8_t response;
	response = SD_send_cmd(CMD24, block, 0x01);
	if (response != 0x00) { CS_HIGH(); return 1; }
	SPI_transmit(0xFF);
	SPI_transmit(0xFE);
	SPI_send_multi(buff, 512);
	SPI_transmit(0xFF);
	SPI_transmit(0xFF);
	response = SPI_receive();
	if ((response & 0x1F) != 0x05) { CS_HIGH(); return 2; }
	while (SPI_receive() == 0x00);
	CS_HIGH();
	SPI_transmit(0xFF);
	return 0;
}

uint8_t SD_readSingleBlock(uint32_t block, uint8_t* buff) {
	uint8_t response;
	uint16_t i;
	response = SD_send_cmd(CMD17, block, 0x01);
	if (response != 0x00) { CS_HIGH(); return 1; }
	for (i = 0; i < 0xFFFF; i++) {
		response = SPI_receive();
		if (response == 0xFE) break;
	}
	if (response != 0xFE) { CS_HIGH(); return 2; }
	for (i = 0; i < 512; i++) buff[i] = SPI_receive();
	SPI_receive();
	SPI_receive();
	CS_HIGH();
	SPI_transmit(0xFF);
	return 0;
}

DSTATUS disk_status(BYTE pdrv) {
	if (pdrv != DEV_MMC) return STA_NOINIT;
	return Stat;
}
DSTATUS disk_initialize(BYTE pdrv) {
	if (pdrv != DEV_MMC) return STA_NOINIT;
	if (SD_init() == 0) {
		Stat &= ~STA_NOINIT;
		return RES_OK;
		} else {
		Stat |= STA_NOINIT;
		return RES_ERROR;
	}
}
DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
	if (pdrv != DEV_MMC) return RES_PARERR;
	if (count == 0) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	for (UINT i = 0; i < count; i++) {
		if (SD_readSingleBlock(sector + i, buff + 512 * i) != 0) {
			return RES_ERROR;
		}
	}
	return RES_OK;
}
DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
	if (pdrv != DEV_MMC) return RES_PARERR;
	if (count == 0) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	for (UINT i = 0; i < count; i++) {
		if (SD_writeSingleBlock(sector + i, buff + 512 * i) != 0) {
			return RES_ERROR;
		}
	}
	return RES_OK;
}
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
	if (pdrv != DEV_MMC) return RES_PARERR;
	switch (cmd) {
		case CTRL_SYNC: *(BYTE*)buff = 0; return RES_OK;
		case GET_SECTOR_SIZE: *(WORD*)buff = 512; return RES_OK;
		case GET_BLOCK_SIZE: *(DWORD*)buff = 1; return RES_OK;
		case GET_SECTOR_COUNT: *(DWORD*)buff = 32768; return RES_OK;
		default: return RES_PARERR;
	}
}
DWORD get_fattime(void) {
	return ((2020UL - 1980UL) << 25) | (1UL << 21) | (1UL << 16);
}
