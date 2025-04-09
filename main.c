#define F_CPU 16000000UL  // 16 MHz clock speed
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#include <avr/io.h>
#include <util/delay.h>

void USART0_Init(unsigned int ubrr) {
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;

	// Enable transmitter
	UCSR0B = (1 << TXEN0);

	// Set frame format: 8 data, 1 stop bit
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART0_Transmit(unsigned char data) {
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));

	// Put data into buffer, sends the data
	UDR0 = data;
}

void USART0_SendString(const char *str) {
	while (*str) {
		USART0_Transmit(*str++);
	}
}

int main(void) {
	USART0_Init(MYUBRR);

	while (1) {
		USART0_SendString("100\n");
		_delay_ms(1);

	}
}
