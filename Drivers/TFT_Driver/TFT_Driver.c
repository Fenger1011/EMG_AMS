/************************************************************
File name: "TFT_driver.c"

Driver for "ITDB02 320 x 240 TFT display module, Version 2"
mounted at "ITDB02 Arduino Mega2560 Shield".
Display controller = ILI 9341.
TFT chip driver is set to 8080 MCU 16-bit interface I by external pins on breakout board

Max. uC clock frequency = 16 MHz (Tclk = 62,5 ns)

Connections:
DB15-DB8:   PORT A
DB7-DB0:    PORT C

RESETx:     PORT G, bit 0
CSx:        PORT G, bit 1
WRx:        PORT G, bit 2
RS (=D/Cx): PORT D, bit 7

Henning Hargaard
Modified Michael Alrøe
************************************************************/


#include <avr/io.h>
#include <stdint.h>
#include <math.h>

#define F_CPU 16000000
#include <util/delay.h>

#include <avr/cpufunc.h>  // _NOP()
#include "TFT_driver.h"

// Data port definitions:
#define DATA_PORT_HIGH PORTA // All 8 bits in PORTA used for high
#define DATA_PORT_LOW  PORTC // All 8 bits in PORTC used for low

// Control port definitions:
#define WR_PORT PORTG
#define WR_BIT 2
#define DC_PORT PORTD
#define DC_BIT  7  //"DC" signal is at the shield called RS
#define CS_PORT PORTG
#define CS_BIT  1
#define RST_PORT PORTG
#define RST_BIT 0


// LOCAL FUNCTIONS /////////////////////////////////////////////////////////////

void pulseWR(void){
	// WR low
	WR_PORT &= ~(1 << WR_BIT);
	
	// t_wrl delay (15 ns), 1 NOP = 62.5ns => no NOP() needed!
	
	// WR high = Latch data
	WR_PORT |= (1 << WR_BIT);
}

// ILI 9341 data sheet, page 238
void WriteCommand(unsigned char command){
	 // Data set up (commands only use the lower byte of the data bus)
	DATA_PORT_LOW = command;
	
	// DC low = command
	DC_PORT &= ~(1 << DC_BIT); 
	
	// CS low = chip selected
	CS_PORT &= ~(1 << CS_BIT);
	
	pulseWR();
	
	// Small delay (62.5ns)
	_NOP();
}

// ILI 9341 data sheet, page 238
void WriteData(uint16_t data){
	// Data set up (Use both high and low of data bus)
	DATA_PORT_HIGH = (data >> 8) & 0xFF; // Get only higher byte of data
	DATA_PORT_LOW = data; // Get only lower byte of data
	
	// DC high = data
	DC_PORT |= (1 << DC_BIT);
	
	// CS low = chip selected
	CS_PORT &= ~(1 << CS_BIT);
	
	pulseWR();
	
	// Small delay (62.5ns)
	_NOP();
}

// PUBLIC FUNCTIONS ////////////////////////////////////////////////////////////

// Initializes (resets) the display
void DisplayInit(){
	// Set data port directions to output
	DDRA = 0xFF;  // DB15–DB8 (PORTA)
	DDRC = 0xFF;  // DB7–DB0 (PORTC)

	// Set control pins to output
	DDRG |= (1 << RST_BIT) | (1 << CS_BIT) | (1 << WR_BIT);
	DDRD |= (1 << DC_BIT);
	
	// Hardware reset (pull RST LOW)
	RST_PORT &= ~(1 << RST_BIT);  // Pull RST low
	_delay_ms(50);
	RST_PORT |= (1 << RST_BIT);   // Release RST high
	_delay_ms(120);
	
	// Software reset
	WriteCommand(0x01);
	_delay_ms(5);
	
	// Display OFF (turn off display)
	WriteCommand(0x28);
		
	//  // 4. Initialization sequence (from ILI9341 datasheet / working reference)

	WriteCommand(0xCF);
	WriteData(0x00);
	WriteData(0xC1);
	WriteData(0x30);

	WriteCommand(0xED);
	WriteData(0x64);
	WriteData(0x03);
	WriteData(0x12);
	WriteData(0x81);

	WriteCommand(0xE8);
	WriteData(0x85);
	WriteData(0x00);
	WriteData(0x78);

	WriteCommand(0xCB);
	WriteData(0x39);
	WriteData(0x2C);
	WriteData(0x00);
	WriteData(0x34);
	WriteData(0x02);

	WriteCommand(0xF7);
	WriteData(0x20);

	WriteCommand(0xEA);
	WriteData(0x00);
	WriteData(0x00);

	WriteCommand(0xC0);    // Power control
	WriteData(0x23);

	WriteCommand(0xC1);    // Power control
	WriteData(0x10);

	WriteCommand(0xC5);    // VCOM control
	WriteData(0x3E);
	WriteData(0x28);

	WriteCommand(0xC7);    // VCOM control
	WriteData(0x86);

	WriteCommand(0x36);    // Memory Access Control
	WriteData(0x48);       // Portrait: MX, BGR

	WriteCommand(0x3A);    // Pixel Format
	WriteData(0x55);       // 16-bit/pixel

	WriteCommand(0xB1);    // Frame rate control
	WriteData(0x00);
	WriteData(0x18);

	WriteCommand(0xB6);    // Display Function Control
	WriteData(0x08);
	WriteData(0x82);
	WriteData(0x27);

	WriteCommand(0xF2);    // Enable 3G
	WriteData(0x00);

	WriteCommand(0x26);    // Gamma Set
	WriteData(0x01);

	WriteCommand(0xE0);    // Positive Gamma
	WriteData(0x0F);
	WriteData(0x31);
	WriteData(0x2B);
	WriteData(0x0C);
	WriteData(0x0E);
	WriteData(0x08);
	WriteData(0x4E);
	WriteData(0xF1);
	WriteData(0x37);
	WriteData(0x07);
	WriteData(0x10);
	WriteData(0x03);
	WriteData(0x0E);
	WriteData(0x09);
	WriteData(0x00);

	WriteCommand(0xE1);    // Negative Gamma
	WriteData(0x00);
	WriteData(0x0E);
	WriteData(0x14);
	WriteData(0x03);
	WriteData(0x11);
	WriteData(0x07);
	WriteData(0x31);
	WriteData(0xC1);
	WriteData(0x48);
	WriteData(0x08);
	WriteData(0x0F);
	WriteData(0x0C);
	WriteData(0x31);
	WriteData(0x36);
	WriteData(0x0F);

	// 5. Exit sleep
	WriteCommand(0x11);
	_delay_ms(120);

	// 6. Turn on display
	WriteCommand(0x29);

	// 7. Ready for pixel writing
	WriteCommand(0x2C);

}

void DisplayOff(){
	WriteCommand(0x28);
}

void DisplayOn(){
	WriteCommand(0x29);
}

void SleepOut()
{
}

void MemoryAccessControl(unsigned char parameter)
{
}

void InterfacePixelFormat(unsigned char parameter)
{
}

void MemoryWrite()
{
}

// Red 0-31, Green 0-63, Blue 0-31
void WritePixel(unsigned char Red, unsigned char Green, unsigned char Blue){
	// Convert 5-6-5 RGB to 16-bit value
	uint16_t color = (Red << 11) | (Green << 5) | Blue;
	WriteData(color);
}

// Set Column Address (0-239), Start > End
void SetColumnAddress(uint16_t Start, uint16_t End){
	WriteCommand(0x2A); // Column address set

	WriteData((Start >> 8) & 0xFF); // Start high byte
	WriteData(Start & 0xFF);        // Start low byte

	WriteData((End >> 8) & 0xFF);   // End high byte
	WriteData(End & 0xFF);          // End low byte
}

// Set Page Address (0-319), Start > End
void SetPageAddress(uint16_t Start, uint16_t End){
	WriteCommand(0x2B); // Page address set

	WriteData((Start >> 8) & 0xFF); // Start high byte
	WriteData(Start & 0xFF);        // Start low byte

	WriteData((End >> 8) & 0xFF);   // End high byte
	WriteData(End & 0xFF);          // End low byte
}

void BackgroundColor(uint8_t Red, uint8_t Green, uint8_t Blue)
{
	uint16_t color = (Red << 11) | (Green << 5) | Blue;

	SetColumnAddress(0, 239);
	SetPageAddress(0, 319);
	WriteCommand(0x2C);  // Memory Write

	for (uint32_t i = 0; i < 240UL * 320UL; i++)
	{
		WriteData(color);
	}
}

void DrawVerticalLine(uint16_t x, uint16_t y_start, uint16_t y_end, uint8_t Red, uint8_t Green, uint8_t Blue)
{
	SetColumnAddress(x, x);
	SetPageAddress(y_start, y_end);
	WriteCommand(0x2C);  // Memory Write

	for (uint16_t y = y_start; y <= y_end; y++)
	{
		WritePixel(Red, Green, Blue);
	}
}

void DrawHorizontalLine(uint16_t y, uint16_t x_start, uint16_t x_end, uint8_t Red, uint8_t Green, uint8_t Blue)
{
	SetColumnAddress(x_start, x_end);
	SetPageAddress(y, y);
	WriteCommand(0x2C);  // Memory Write

	for (uint16_t x = x_start; x <= x_end; x++)
	{
		WritePixel(Red, Green, Blue);
	}
}

void InitCoordinate() {
	// 1. Fill screen with white (RGB = 255, 255, 255)
	BackgroundColor(31, 63, 31);  // Max R, G, B for RGB565 white

	// 2. Draw vertical black line (R=0, G=0, B=0)
	DrawVerticalLine(120, 0, 319, 0, 0, 0);  // Center X

	// 3. Draw horizontal black line
	DrawHorizontalLine(260, 0, 239, 0, 0, 0);  // Center Y
}

void DrawEMG(uint8_t sample, uint16_t x)
{
	// Map 0–255 sample to Y screen coordinate (0 = bottom, 239 = top)
	// 239 - (sample * 240 / 256) flips Y axis (so 0 = bottom)
	uint16_t y = 239 - ((sample * 240UL) / 256);

	SetColumnAddress(y, y);
	SetPageAddress(x,x);
	WriteCommand(0x2C);  // Memory Write
	WritePixel(31, 0, 0);  // Dark blue pixel
}