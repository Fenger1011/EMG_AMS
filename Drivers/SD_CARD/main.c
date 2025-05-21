#include <avr/io.h>
#define F_CPU 16000000UL  // Or your actual clock in Hz, e.g., 16000000UL for Mega 2560
#include <util/delay.h>

#include "ff.h"
#include "diskio.h"
#include "SD_Driver.h"

FATFS fs;
FIL file;
UINT bw;

int main(void) {
	// Wait for SD card to power up
	_delay_ms(100);

	// Mount
	if (f_mount(&fs, "", 1) != FR_OK) while (1);

	// Open/create a file
	if (f_open(&file, "test.txt", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) while (1);

	// Write something
	const char text[] = "Hello, Mega2560 SD + FatFS!\r\n";
	if (f_write(&file, text, sizeof(text)-1, &bw) != FR_OK || bw != sizeof(text)-1) while (1);

	// Close file
	f_close(&file);

	// Done, loop forever
	while (1);
}
