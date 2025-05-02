#include <avr/io.h>

// Initializes USART
void USART0_Init(unsigned int ubrr) {
	
	// Need to set the Baud rate registers for USART
	UBRR0H = (unsigned char)(ubrr >> 8); // Holds high byte (15-8)
	UBRR0L = (unsigned char)ubrr; // Holds low byte (7-0)
	
	UCSR0B = (1 << TXEN0); // UCSR0B is USART control and status register B, (1<<TXEN0) enables transmission
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // UCSR0C is USART control and status register C, this sets USART to 8-bit mode, USBS0=0 mean 1 stop bit {8N1 mode}
}

// Transmits 1 byte of data
void USART0_Transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0))); // Loops until UDRE0 in UCSR0A becomes 1 (When UDRE0=1, status flag is low, we can transmit again)
	UDR0 = data; // Set send/receive USART register to data
}

void USART0_SendString(const char *str) {
	while (*str) USART0_Transmit(*str++); // While characters in string (*str) to transmit, send character, in C strings are null-terminated.
}