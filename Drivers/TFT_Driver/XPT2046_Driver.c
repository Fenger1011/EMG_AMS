#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "TFT_Driver.h"

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

// ========== Private Variables ==========
static uint16_t x_min, x_max = 0;	// For raw x-coordinate during calibration
static uint16_t y_min, y_max = 0;	// For raw y-coordinate during calibration

// ========== Global Variables ==========
volatile uint8_t touch_triggered = 0;

// ========== UART Functions ==========
void uart_init() {
	UBRR0H = 0;
	UBRR0L = 103;  // 9600 baud at 16MHz
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_tx(char c) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void uart_print(const char* str) {
	while (*str) uart_tx(*str++);
}

void uart_print_num(uint16_t num) {
	char buf[7];
	snprintf(buf, sizeof(buf), "%u", num);
	uart_print(buf);
}

// ========== SPI Bit-Bang Functions ==========
void spi_write(uint8_t data) {
	for (int i = 7; i >= 0; i--) {
		if (data & (1 << i)) SET(D_IN_PORT, D_IN_PIN);
		else CLR(D_IN_PORT, D_IN_PIN);

		SET(D_CLK_PORT, D_CLK_PIN);
		_delay_us(1);
		CLR(D_CLK_PORT, D_CLK_PIN);
		_delay_us(1);
	}
}

uint16_t spi_read12() {
	uint16_t result = 0;
	for (int i = 0; i < 12; i++) {
		result <<= 1;
		SET(D_CLK_PORT, D_CLK_PIN);
		_delay_us(1);
		if (READ(D_OUT_PINR, D_OUT_PIN)) result |= 1;
		CLR(D_CLK_PORT, D_CLK_PIN);
		_delay_us(1);
	}
	return result;
}

// ========== Interrupt Setup ==========
// External interrupt on pin PE4 = IRQ pin
void init_interrupt() {
	
	// Trigger interrupt on PE4 falling edge
	EICRB |=  (1 << ISC41);
	EICRB &= ~(1 << ISC40);
	
	
	EIFR  |=  (1 << INTF4);   // clear any pending flag
	EIMSK |=  (1 << INT4);    // enable INT4
}

// ========== ISR: Touch Trigger ==========
ISR(INT4_vect) {
	touch_triggered = 1; // Flag for when touch triggered
}

// ========== Pin Initialization ==========
void init_pins() {
	// Outputs
	SET(D_CS_PORT, D_CS_PIN);		// Set CS = LOW (do this before setting as output!)
	D_CS_DDR |= (1 << D_CS_PIN);	// CS
	D_CLK_DDR |= (1 << D_CLK_PIN);	// CLK
	D_IN_DDR  |= (1 << D_IN_PIN);	// MOSI

	// Inputs
	DDRE &= ~(1 << D_OUT_PIN);		// MISO
	DDRE &= ~(1 << D_IRQ_PIN);		// IRQ
	PORTE |=  (1 << D_IRQ_PIN);	    // pull-up on IRQ (active low, ISR trigger on falling edge)
}


// ============= Get Raw Coordinates ===============
void GetRawCoordinates(uint16_t* x_raw, uint16_t* y_raw) {
	
	CLR(D_CS_PORT, D_CS_PIN);	// Pull CS LOW
	//while (READ(D_IRQ_PINR, D_IRQ_PIN));  // Wait for press (IRQ active LOW)

	spi_write(0x90);			// Ask for X-coordinate
	*x_raw = spi_read12();		// Store X-coordinate in pointer
	spi_write(0xD0);			// Ask for Y-coordinate
	*y_raw = spi_read12();		// Store Y-coordinate in pointer
	SET(D_CS_PORT, D_CS_PIN);	// Pull CS HIGH (deselect chip)
	
	while (!READ(D_IRQ_PINR, D_IRQ_PIN));  // Wait for release
}


// ============= Calibrate Touchscreen =============
void CalibrateTouchScreen() {
	// Show square in upper leftmost corner
	DrawSquare(0, 220, 20, 31, 62, 0);

	uint16_t x, y;

	// Wait for first touch (upper left)
	while (!touch_triggered);

	if (!READ(D_IRQ_PINR, D_IRQ_PIN)) {
		GetRawCoordinates(&x, &y);
		x_min = x;
		y_min = y;

		uart_print("x_min: ");
		uart_print_num(x_min);
		uart_print(", y_min: ");
		uart_print_num(y_min);
		uart_print("\r\n");

		while (!READ(D_IRQ_PINR, D_IRQ_PIN));  // Wait for release
		
		BackgroundColor(31, 62, 31);
	}
	
	// Set = 0 here and not start of while statement, because the interrupt will run again too soon.
	touch_triggered = 0;
	
	_delay_ms(20); // Debounce

	// Show square in bottom rightmost corner
	DrawSquare(320, 0, 20, 31, 62, 0);

	// Wait for second touch (bottom right)
	while (!touch_triggered);
	touch_triggered = 0;

	if (!READ(D_IRQ_PINR, D_IRQ_PIN)) {
		GetRawCoordinates(&x, &y);
		x_max = x;
		y_max = y;

		uart_print("x_max: ");
		uart_print_num(x_max);
		uart_print(", y_max: ");
		uart_print_num(y_max);
		uart_print("\r\n");

		while (!READ(D_IRQ_PINR, D_IRQ_PIN));  // Wait for release
	}
}



// ================= Get Calibrated X/Y coordinates =====================
void GetCoordinates(uint16_t* x, uint16_t* y) {
	uint16_t x_raw, y_raw;
	GetRawCoordinates(&x_raw, &y_raw);
	
	
	if (x_max == x_min || y_max == y_min) {
		*x = 0;
		*y = 0;
		return; // Avoid division by zero
	}

	// Map raw values to screen coordinates
	int32_t x_temp = ((int32_t)(x_raw - x_min)) * SCREEN_WIDTH / (x_max - x_min);
	int32_t y_temp = ((int32_t)(y_raw - y_min)) * SCREEN_HEIGHT / (y_max - y_min);

	// Make sure calculated x and y are within borders
	// If not make equal to borders
	if (x_temp < 0) x_temp = 0;
	if (x_temp > SCREEN_WIDTH) x_temp = SCREEN_WIDTH;
	if (y_temp < 0) y_temp = 0;
	if (y_temp > SCREEN_HEIGHT) y_temp = SCREEN_HEIGHT;

	*x = (uint16_t)x_temp;
	*y = (uint16_t)y_temp;
}




// ========== Main ==========
int main(void) {
	uart_init();
	init_pins();
	init_interrupt();
	sei();
	
	// Values for holding x and y coordinates
	uint16_t x, y;
	
	CalibrateTouchScreen();
	
	while (1) {
	}
}