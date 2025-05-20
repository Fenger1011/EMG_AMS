#ifndef TOUCH_DRIVER_H
#define TOUCH_DRIVER_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

// ========== Screen Dimensions ==========
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

// ========== Pin Definitions (XPT2046, bit-banged SPI) ==========
#define D_CLK_PORT  PORTH
#define D_CLK_DDR   DDRH
#define D_CLK_PIN   PH3

#define D_CS_PORT   PORTE
#define D_CS_DDR    DDRE
#define D_CS_PIN    PE3

#define D_IN_PORT   PORTG
#define D_IN_DDR    DDRG
#define D_IN_PIN    PG5

#define D_OUT_PINR  PINE
#define D_OUT_PIN   PE5

#define D_IRQ_PINR  PINE
#define D_IRQ_PIN   PE4

// ========== Helper Macros ==========
#define SET(port, pin)    ((port) |= (1 << (pin)))
#define CLR(port, pin)    ((port) &= ~(1 << (pin)))
#define READ(pinr, pin)   ((pinr) & (1 << (pin)))

// ========== External Variables ==========
extern volatile uint8_t touch_triggered;

// ========== Function Prototypes ==========

// UART
void uart_init(void);
void uart_tx(char c);
void uart_print(const char* str);
void uart_print_num(uint16_t num);

// SPI Bit-Bang
void spi_write(uint8_t data);
uint16_t spi_read12(void);

// Interrupts
void InitTouchInterrupt(void);

// Touch Handling
void init_pins(void);
void GetRawCoordinates(uint16_t* x_raw, uint16_t* y_raw);
void CalibrateTouchScreen(void);
void GetCoordinates(uint16_t* x, uint16_t* y);

#endif // TOUCH_DRIVER_H
