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
#define MYUBRR (F_CPU/16/BAUD - 1)
#define VREF 5000
#define BUFFER_SIZE 480

// EMG buffer and flags
volatile uint16_t emg_samples[BUFFER_SIZE];
volatile uint16_t emg_index = 0;
volatile uint8_t emg_buffer_full = 0;

// EMG processing variables
uint16_t x = 319;
uint16_t rms_adc = 0;
uint32_t rms_mv = 0;
uint16_t threshold = 200;
uint16_t overThreshold = 0;
uint16_t underThreshold = 0;
char buffer[10];

// Touch flag (must be defined in your touch driver)
extern volatile uint8_t touch_triggered;

// Screen states
typedef enum {
	STATE_SCREEN_A,
	STATE_SCREEN_B
} ScreenState;

ScreenState current_state = STATE_SCREEN_A;

/******************************************************* PWM *************************************************************/
void pwm_init(void) {
	DDRH |= (1 << PH5);
	TCCR4A = (1 << COM4C1);
	TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41) | (1 << CS40);
	ICR4 = 4999;
}

void pwm_set_duty(uint16_t duty_percent) {
	OCR4C = (uint32_t)ICR4 * duty_percent / 100;
}

/******************************************************* ADC *************************************************************/
void adc_init(void) {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	sei();
	ADCSRA |= (1 << ADSC);
}

/******************************************************* ISR *************************************************************/
ISR(ADC_vect) {
	emg_samples[emg_index++] = ADC;
	if (emg_index >= BUFFER_SIZE) {
		emg_index = 0;
		emg_buffer_full = 1;
	}
	ADCSRA |= (1 << ADSC);
}

/************************************************ RMS Calculation ********************************************************/
uint16_t calculate_RMS(void) {
	uint32_t sum = 0;
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) sum += emg_samples[i];
	uint16_t mean = sum / BUFFER_SIZE;

	uint32_t sum_squares = 0;
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		int16_t centered_sample = (int16_t)emg_samples[i] - (int16_t)mean;
		sum_squares += (uint32_t)centered_sample * centered_sample;
	}
	uint32_t mean_square = sum_squares / BUFFER_SIZE;
	return (uint16_t)sqrt((double)mean_square);
}

/************************************************ Motor Control **********************************************************/
void closeHand(void) {
	pwm_set_duty(6);
	_delay_ms(500);
	pwm_set_duty(0);
}

void openHand(void) {
	pwm_set_duty(9);
	_delay_ms(475);
	pwm_set_duty(0);
}

/************************************************ Screen Logic ***********************************************************/
void ScreenA(void) {
	if (emg_buffer_full) {
		emg_buffer_full = 0;
		rms_adc = calculate_RMS();
		rms_mv = (uint32_t)rms_adc * VREF / 1023;
		itoa(rms_mv, buffer, 10);
		USART0_SendString(buffer);
		USART0_Transmit('\n');

		uint16_t mapped_sample = ((rms_mv * 239UL) / 2000);
		DrawEMG(mapped_sample, x);
		x -= 3;
		if (x <= 1) {
			x = 319;
			InitCoordinate();
		}

		if (rms_mv >= threshold) {
			overThreshold++;
			if (overThreshold == 3) {
				PORTB |= (1 << PB7);
				closeHand();
				underThreshold = 0;
			}
		}

		if (rms_mv <= threshold) {
			underThreshold++;
			if (underThreshold == 5) {
				PORTB &= ~(1 << PB7);
				openHand();
				overThreshold = 0;
			}
		}
	}
}

void ScreenB(void) {
	// Static or idle screen — draw once on entry only
	BackgroundColor(31, 0, 0);  // Red background
	
	// Wait for press to go low and then high = 1 press
	while ( READ(D_IRQ_PINR, D_IRQ_PIN) ) { 
		while ( !READ(D_IRQ_PINR, D_IRQ_PIN) ) { /* idle */ } 
		}

	
}

/************************************************ Main *******************************************************************/
int main(void) {
	USART0_Init(MYUBRR);
	adc_init();
	pwm_init();
	DisplayInit();
	InitTouchInterrupt();    // Still needed if your driver uses it
	sei();
	init_pins();
	CalibrateTouchScreen();
	DDRB |= (1 << PB7);      // Debug LED pin

	current_state = STATE_SCREEN_A;
	x = 319;

	// 1) Draw Screen A once on startup:
	InitCoordinate();

	for (;;) {
		switch (current_state) {
			case STATE_SCREEN_A:
			// — Run ScreenA() continuously until we see a touch —
			while ( READ(D_IRQ_PINR, D_IRQ_PIN) ) {
				// IRQ pin HIGH ? no touch
				ScreenA();
			}

			// — We just saw IRQ pin LOW ? finger down —
			_delay_ms(50);    // debounce for press
			// wait for finger lift
			while ( !READ(D_IRQ_PINR, D_IRQ_PIN) );
			_delay_ms(50);    // debounce for release

			// — Toggle to B —
			current_state = STATE_SCREEN_B;
			ScreenB();       // draw B once
			break;


			case STATE_SCREEN_B:
			// — Run ScreenB() continuously until touch —
			while ( READ(D_IRQ_PINR, D_IRQ_PIN) ) {
				ScreenB();
			}

			_delay_ms(50);
			while ( !READ(D_IRQ_PINR, D_IRQ_PIN) );
			_delay_ms(50);

			// — Toggle back to A —
			current_state = STATE_SCREEN_A;
			InitCoordinate();  // redraw axes
			x = 319;           // reset graph position
			overThreshold = underThreshold = 0;
			break;
		}
	}
}


