#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "USART_Driver.h"
#include "TFT_driver.h"
#include "XPT2046_Driver.h"

#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD - 1) // Set baud rate for UART

#define VREF 5000								// ADC reference in millivolts
#define BUFFER_SIZE 480							// 50 ms window at ~9600 Hz ADC sampling frequency
volatile uint16_t emg_samples[BUFFER_SIZE];		// Buffer for holding EMG samples
volatile uint16_t emg_index = 0;				// For knowing buffer index
volatile uint8_t emg_buffer_full = 0;			// Flag for when sample buffer is full and calculations should be triggered


/*******************************************************PWM*************************************************************/
// Initializes PWM for motor control
// PWM frequency is 50 Hz (20 ms period) with 16 MHz clock and prescaler 64
void pwm_init(void) {
	// Set PH5 (OC4C, Arduino Pin 8) as output
	DDRH |= (1 << PH5);

	// Fast PWM with ICR4 as TOP, non-inverted PWM on OC4C
	TCCR4A = (1 << COM4C1);              // Clear OC4C on Compare Match
	TCCR4B = (1 << WGM43) | (1 << WGM42) // Fast PWM, mode 14 (ICR TOP)
	| (1 << CS41) | (1 << CS40);  // Prescaler 64

	ICR4 = 4999;  // TOP ? 50 Hz PWM
}

// Set duty cycle for motor control
// duty_percent is a value between 0 and 100
void pwm_set_duty(uint16_t duty_percent) {
	OCR4C = (uint32_t)ICR4 * duty_percent / 100;
}

/***********************************************************************************************************************/


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
// ADC ISR for reading EMG samples, storing in buffer and raising flag when buffer full (50 ms)
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
// Calculate RMS value from EMG samples when buffer is full
uint16_t calculate_RMS(void) {
	
	// Summing all samples in buffer
	uint32_t sum = 0;
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		sum += emg_samples[i];
	}
	
	// Calculate mean
	uint16_t mean = sum / BUFFER_SIZE;
	
	// Subtract mean and calculate squares (Equivalent for mean of squares)
	uint32_t sum_squares = 0;
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		int16_t centered_sample = (int16_t)emg_samples[i] - (int16_t)mean; // Removing DC component makes RMS calculation of true AC component
		sum_squares += (uint32_t)centered_sample * (uint32_t)centered_sample;
	}
	
	// Mean square
	uint32_t mean_square = sum_squares / BUFFER_SIZE;
	
	// Compute square root
	uint16_t rms = (uint16_t)sqrt((double)mean_square);

	return rms; 
}
/************************************************************************************************************************/

/************************************************MOTOR MOVEMENTS******************************************************* */
// Close hand
void closeHand(void){
	pwm_set_duty(6); // Move motor
	_delay_ms(500); // Let motor move
	pwm_set_duty(0); // Stop motor
}

// Open hand
void openHand(void){
	pwm_set_duty(9); // Move motor
	_delay_ms(475); // Let motor move
	pwm_set_duty(0); // Stop motor
}
/************************************************************************************************************************/



int main(void) {

	USART0_Init(MYUBRR);	// Initialize UART
	adc_init();				// Initialize ADC
	pwm_init();				// Initialize PWM
	DisplayInit();			// Initialize TFT
	InitTouchInterrupt();	// Initialize for falling edge interrupt
	sei();					// Global interrupts
	init_pins();			// Pins for bit-banged SPI
	
	CalibrateTouchScreen();
	// InitCoordinate();	// Display coordinate system
	
	uint16_t x_calibrated, y_calibrated;
	GetCoordinates(&x_calibrated, &y_calibrated); // Test if different from CalibrateTouchScreen();
	
	uint16_t x = 319;				// Start coordinate for x-axis (Helt til venstre)
	uint16_t rms_adc = 0;
	uint32_t rms_mv = 0;
	uint16_t threshold = 200;		//Threshold in mV for when to move motor
	uint16_t overThreshold = 0;
	uint16_t underThreshold = 0;
	char buffer[10];				// Enough for millivolt values (max "5000\0")
	
	DDRB |= (1 << PB7);	// Set pin 13 (PB7) as output for debugging (LED)
	
	
	
	while (1) 
	{
		
		/*
		// Check if EMG buffer is full (ISR sets this flag)
		if (emg_buffer_full) {
			emg_buffer_full = 0;  // Clear flag

			// Calculate RMS from EMG samples buffer
			rms_adc = calculate_RMS();

			// Convert ADC value to millivolts
			rms_mv = (uint32_t)rms_adc * VREF / 1023;

			// Convert rms_mv to string
			itoa(rms_mv, buffer, 10);  // Convert to decimal string

			USART0_SendString(buffer);
			
			// Send newline
			USART0_Transmit('\n');
			
			// Scale ADC output into TFT y-axis range
			uint16_t mapped_sample = ((rms_mv * 239UL) / 2000);


			DrawEMG(mapped_sample, x);
			
			// increment x-axis position
			x--;
			
			// If x has reached 0 (rightmost) restart screen with coordinate
			if (x <= 0) {
				x = 319;
				InitCoordinate(); // Clear and redraw axis
			}
			
			// If 50ms window RMS value above threshold
			if (rms_mv >= threshold) {
				overThreshold++;
				
				if (overThreshold == 3) { // If RMS above threshold for 3 full windows (150 ms)
					PORTB |= (1 << PB7); // Turn ON LED (debugging)
					closeHand(); // Move motor
					underThreshold = 0; // Reset under threshold => Could otherwise have old values and trigger open hand very fast
				}
			}

			// If 50ms window RMS value below threshold
			if (rms_mv <= threshold) {
				underThreshold++;
				
				if (underThreshold == 5) { // If RMS below threshold for 5 full windows (250ms)
					PORTB &= ~(1 << PB7); // Turn OFF LED (debugging)
					openHand(); // Move motor
					overThreshold = 0; // Reset over threshold threshold => Could otherwise have old values and trigger close hand very fast
				}
			}
		}*/
	} 
}
