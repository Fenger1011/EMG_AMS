#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "USART_Driver.h"

#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD - 1) // Set baud rate for UART

#define VREF 5000								// ADC reference in millivolts
#define BUFFER_SIZE 480							// 50 ms window at ~9600 Hz ADC sampling frequency
volatile uint16_t emg_samples[BUFFER_SIZE];		// Buffer for holding EMG samples
volatile uint16_t emg_index = 0;				// For knowing buffer index
volatile uint8_t emg_buffer_full = 0;			// Flag for when sample buffer is full and calculations should be triggered


/*******************************************************ADC*************************************************************/
// Initializes ADC
void adc_init(void) {
	ADMUX  = (1 << REFS0);									// AVcc as ref
	ADCSRA = (1 << ADEN)  | (1 << ADIE)						// Enable ADC + interrupt
		   | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// Prescaler 128
	sei();													// Global interrupts on
	ADCSRA |= (1 << ADSC);									// Kick off first conversion
}
/***********************************************************************************************************************/

/*******************************************************ISR*************************************************************/
ISR(ADC_vect) {
	emg_samples[emg_index++] = ADC;

	if (emg_index >= BUFFER_SIZE) {
		emg_index = 0;
		emg_buffer_full = 1;  // set flag to signal main
	}

	ADCSRA |= (1 << ADSC); // start next conversion
}
/***********************************************************************************************************************/

/*****************************************SIGNAL CONDITIONING***********************************************************/
uint16_t calculate_RMS(void) {

	uint32_t sum = 0;
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		sum += emg_samples[i];
	}
	uint16_t mean = sum / BUFFER_SIZE;

	uint32_t sum_squares = 0;
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		int16_t centered_sample = (int16_t)emg_samples[i] - (int16_t)mean;
		sum_squares += (uint32_t)centered_sample * (uint32_t)centered_sample;
	}

	uint32_t mean_square = sum_squares / BUFFER_SIZE;
	uint16_t rms = (uint16_t)sqrt((double)mean_square);

	return rms;  // 0..1023 ? ADC range RMS (centered)
}
/************************************************************************************************************************/

int main(void) {

	USART0_Init(MYUBRR);
	adc_init();

	uint16_t rms_adc = 0;
	uint32_t rms_mv = 0;
	uint16_t threshold = 1000; //Threshold in mV for when to move motor
	
	// Set pin 13 (PB7) as output for debugging
	DDRB |= (1 << PB7);

	while (1) {

		// Check if EMG buffer is full (ISR sets this flag)
		if (emg_buffer_full) {
			emg_buffer_full = 0;  // Clear flag

			// Calculate RMS from EMG samples buffer
			rms_adc = calculate_RMS();

			// Convert ADC value to millivolts
			rms_mv = (uint32_t)rms_adc * VREF / 1023;

			// If 50ms window RMS value above 
			if (rms_mv >= 1000){
				PORTB |= (1 << PB7); // Turn ON LED
			} else {
				PORTB &= ~(1 << PB7); // Turn OFF LED
			}
			
		}

		// Other background tasks (optional)
	}
}

