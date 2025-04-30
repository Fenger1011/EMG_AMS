#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD - 1)
#define VREF 5000         // in millivolts
#define BUFFER_SIZE 480   // 50 ms at ~9600 Hz

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

void adc_init(void) {
	ADMUX = (1 << REFS0);  // AVcc reference
	ADCSRA = (1 << ADEN)  |
	(1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Prescaler 128
}

uint16_t adc_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

int main(void) {
	USART0_Init(MYUBRR);
	adc_init();

	uint16_t adc_val;
	uint32_t voltage_mv;
	int32_t ac_sample;
	uint32_t sum_squares;
	uint16_t rms_mv;
	char buffer[16];
	
	while (1) {
		sum_squares = 0;

		for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
			adc_val = adc_read(0);  // Fast sampling at ~9.6 kHz
			voltage_mv = ((uint32_t)adc_val * VREF) / 1023;
			ac_sample = (int32_t)voltage_mv - 2500;
			sum_squares += (uint32_t)(ac_sample * ac_sample);
		}

		rms_mv = (uint16_t)sqrt(sum_squares / BUFFER_SIZE);

		itoa(rms_mv, buffer, 10);
		USART0_SendString(buffer);
		USART0_SendString("\r\n");

		// Optional small pause between batches (not needed for sampling)
		// _delay_ms(1);
		
		
	}
}
