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
	SPI_DDR |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS);	// MOSI, SCK, SS as OUTPUT
	SPI_DDR &= ~(1<<DD_MISO);						// MISO as INPUT
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);	// Enable SPI in master, low speed
	SPSR = 0;										// No double speed
	CS_HIGH();										// Chip-select high --> afkobl SD-kortet
}

uint8_t SPI_transmit(uint8_t data) {
	SPDR = data;					// Skriv til data register
	while (!(SPSR & (1<<SPIF)));	// Vent til byte er sendt/modtaget
	return SPDR;					// Læs indkommende byte
}

uint8_t SPI_receive(void) {
	return SPI_transmit(0xFF);	// Læser indkommende data ved at sende 0xFF --> SPI er fuld duplex
}

void SPI_send_multi(const uint8_t* data, uint16_t len) {
	while (len--) SPI_transmit(*data++);
}

void SPI_receive_multi(uint8_t* data, uint16_t len) {
	while (len--) *data++ = SPI_receive();
}

// Sender kommmando og venter på svar
// Følger SD SPI-protokollen som definerer at kommandoer skal sendes som:
//      1 byte: kommando (med startbit)
//		4 byte: argument
//		1 byte: CRC
//		--> svar returneres som én byte (R1)
uint8_t SD_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
	uint8_t response, retry = 0;
	CS_LOW();								// vælg SD-kort (chip select LOW)
	SPI_transmit(0xFF);						// lead-in (sikrer korrekt timing)
	SPI_transmit(0x40 | cmd);				// kommando-byte (startbit + index)
	
	// SD-kortet forventer 32-bit argument, sendt MSB først
	SPI_transmit((uint8_t)(arg >> 24));
	SPI_transmit((uint8_t)(arg >> 16));
	SPI_transmit((uint8_t)(arg >> 8));
	SPI_transmit((uint8_t)arg);
	SPI_transmit(crc);
	
	// Vent for respons
	do {
		response = SPI_receive();
	} while ((response & 0x80) && ++retry < 10); // R1 har bestemt format && prøver 10 gange
	return response;
}

// Initialiserer SD-kort
// Bringer det fra idle til ready state
// Sætter SPI-hastigheden op igen
uint8_t SD_init(void) {
	uint8_t i, response;
	SPI_init();		// Har lav hastighed defineret
	CS_HIGH();		// Frakobler SD-kortet
	
	// Sender 10 'dummy-bytes' som vækker kortet (74 SPI clocks)
	for (i = 0; i < 10; i++) SPI_transmit(0xFF);
	
	// Sender CMD0 (GO_IDLE_STATE) op til 10 gange (gør SD-kortet idle)
	for (i = 0; i < 10; i++) {
		response = SD_send_cmd(CMD0, 0, 0x95);
		CS_HIGH(); 
		SPI_transmit(0xFF);
		
		if (response == 0x01) break;
		_delay_ms(10);
	}
	
	// Hvis kortet ikke gik i idle state (efter 10 forsøg) --> returner fejl 1
	if (response != 0x01) return 1;
	
	// Send CMD8 (SEND_IF_COND) kommando --> tjekker SD version og om det understøttet 3.3V
	response = SD_send_cmd(CMD8, 0x1AA, 0x87);
	
	// Efter CMD8 svarer SD-kortet med R7 respons:
			// 1 byte R1-status
			// 4 bytes ekstra
	for (i = 0; i < 4; i++) SPI_receive();
	
	// clean exit
	CS_HIGH(); SPI_transmit(0xFF);
	
	// Tjekker svaret fra CMD8 --> hvis ikke 0x01 returneres fejl 2
	if (response != 0x01) return 2;
	
	// Initialiseringsloop kører 100 gange --> SD skal returnere 0x00 for at være klar til brug
	for (i = 0; i < 100; i++) {
		SD_send_cmd(CMD55, 0, 0x65);
		response = SD_send_cmd(ACMD41, 0x40000000, 0x77);
		CS_HIGH(); SPI_transmit(0xFF);
		if (response == 0x00) break;
		_delay_ms(10);
	}
	
	// Hvis kortet ikke er klar (ingen 0x00 respons) returner fejl 3
	if (response != 0x00) return 3;
	
	// Sender CMD58
	response = SD_send_cmd(CMD58, 0, 0);
	for (i = 0; i < 4; i++) SPI_receive();
	
	// Afslutter med dummy bytes
	CS_HIGH(); SPI_transmit(0xFF);
	
	// Øger SPI hastighed på MEGA
	SPCR = (1<<SPE)|(1<<MSTR);
	
	// Returnerer 0 hvis alt lykkedes
	return 0;
}

// Bruges til at skrive én enkelte block (512b) til SD-kortet via SPI
uint8_t SD_writeSingleBlock(uint32_t block, const uint8_t* buff) {
	uint8_t response;
	
	// CMD24 er kommandoen WRITE_SINGLE_BLOCK
	response = SD_send_cmd(CMD24, block, 0x01);
	
	// Hvis svaret ikke er 0x00 afviste SD kommandoen, returner fejl 1
	if (response != 0x00) { CS_HIGH(); return 1; }
		
	// SPI transmitterer
	SPI_transmit(0xFF);			// lead-in
	SPI_transmit(0xFE);			// 'data-start' token
	SPI_send_multi(buff, 512);	// Sender 512 bytes fra buff via SPI
	SPI_transmit(0xFF);			// dummy CRC	
	SPI_transmit(0xFF);			// dummy CRC
	
	// SD-kortet svarer på ovenstående med data response token
	response = SPI_receive();
	
	// Kun hvis data token's laveste bits = 0x05 er 'data accepted'
	if ((response & 0x1F) != 0x05) { CS_HIGH(); return 2; }
		
	// Så længe SD-kort sender 0x00 er det 'busy' --> skriver internt til dets flash
	while (SPI_receive() == 0x00);
	
	// Afslut med CS high og dummy bytes
	CS_HIGH();
	SPI_transmit(0xFF);
	
	// Return 0 = alt gik godt :)
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
	
	// Hvis vi ikke modtager start token --> return fejl 2
	if (response != 0xFE) { CS_HIGH(); return 2; }
		
	// Læs 512 bytes fra SD kort og gem i buff
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
