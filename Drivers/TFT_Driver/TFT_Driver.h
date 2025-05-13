#ifndef TFT_DRIVER_H
#define TFT_DRIVER_H

#include <avr/io.h>
#include <stdint.h>	

// Data port definitions (16-bit data bus)
#define DATA_PORT_HIGH PORTA  // DB15–DB8
#define DATA_PORT_LOW  PORTC  // DB7–DB0

// Control signals
#define WR_PORT  PORTG
#define WR_BIT   2

#define DC_PORT  PORTD
#define DC_BIT   7  // RS (D/CX) = Data/Command Select

#define CS_PORT  PORTG
#define CS_BIT   1

#define RST_PORT PORTG
#define RST_BIT  0

// Public API Functions

void DisplayInit(void);
void DisplayOn(void);
void DisplayOff(void);
void SleepOut(void);

void WriteCommand(uint8_t command);
void WriteData(uint16_t data);
void MemoryAccessControl(uint8_t parameter);
void InterfacePixelFormat(uint8_t parameter);
void MemoryWrite(void);
void WritePixel(uint8_t Red, uint8_t Green, uint8_t Blue);

void SetColumnAddress(uint16_t Start, uint16_t End);
void SetPageAddress(uint16_t Start, uint16_t End);
void FillRectangle(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height,
uint8_t Red, uint8_t Green, uint8_t Blue);
void BackgroundColor(uint8_t Red, uint8_t Green, uint8_t Blue);
void DrawVerticalLine(uint16_t x, uint16_t y_start, uint16_t y_end, uint8_t Red, uint8_t Green, uint8_t Blue);
void DrawHorizontalLine(uint16_t y, uint16_t x_start, uint16_t x_end, uint8_t Red, uint8_t Green, uint8_t Blue);
void InitCoordinate();
void DrawEMG(uint8_t sample, uint16_t PageAddress);

#endif // TFT_DRIVER_H
