#define F_CPU 16000000UL

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define VREF 5000  // in millivolts (5.00V)

void adc_init(void) {
	ADMUX = (1 << REFS0);  // AVcc with external capacitor at AREF
	ADCSRA = (1 << ADEN)  | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC enable, prescaler 128
}

uint16_t adc_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);  // Select channel 0–7
	ADCSRA |= (1 << ADSC);  // Start conversion
	while (ADCSRA & (1 << ADSC));  // Wait
	return ADC;
}

void USART0_Init(unsigned int ubrr) {
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART0_Transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void USART0_SendString(const char *str) {
	while (*str) USART0_Transmit(*str++);
}

int main(void) {
	USART0_Init(MYUBRR);
	adc_init();

	char buffer[20];

	while (1) {
		uint16_t adc_val = adc_read(0);  // Read from A0
		uint32_t voltage_mv = ((uint32_t)adc_val * VREF) / 1023;  // in millivolts
		
		uint32_t ac_coupled = voltage_mv - 2500; // Remove 2.5V DC bias
		int envelope = abs(ac_coupled);

		// Format string: e.g., "Voltage: 2480 mV\r\n"
		itoa(envelope, buffer, 10);
		USART0_SendString("Voltage: ");
		USART0_SendString(buffer);
		//USART0_SendString(" mV\r\n");

		_delay_ms(50);
	}
}
