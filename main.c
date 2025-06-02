#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <string.h>

#include "USART_Driver.h"
#include "TFT_driver.h"
#include "XPT2046_Driver.h"

#include "ff.h"
#include "diskio.h"
#include "SD_Driver.h"

FATFS fs;
FIL file;
UINT bw;

#define BAUD         9600
#define MYUBRR       (F_CPU/16/BAUD - 1)
#define VREF         5000
#define BUFFER_SIZE  480

// EMG buffer and flags
volatile uint16_t emg_samples[BUFFER_SIZE];
volatile uint16_t emg_index       = 0;
volatile uint8_t  emg_buffer_full = 0;

// EMG processing variables
uint16_t x            = 319;
uint16_t rms_adc      = 0;
uint32_t rms_mv       = 0;
uint16_t threshold    = 200;
uint16_t overThreshold  = 0;
uint16_t underThreshold = 0;
char buffer[12];

// Touch flag (comes from your touch driver)
extern volatile uint8_t touch_triggered;

// Screen states
typedef enum {
	STATE_SCREEN_A,
	STATE_SCREEN_B
} ScreenState;

ScreenState current_state = STATE_SCREEN_A;

// Blink flag (set in Timer1 interrupt)
volatile uint8_t blink_flag = 0;

/******************************************************* PWM *************************************************************/
void pwm_init(void) {
	DDRH |= (1 << PH5);
	TCCR4A = (1 << COM4C1);
	TCCR4B = (1 << WGM43) | (1 << WGM42)
	| (1 << CS41) | (1 << CS40);
	ICR4 = 4999;
}

void pwm_set_duty(uint16_t duty_percent) {
	OCR4C = ((uint32_t)ICR4 * duty_percent) / 100;
}

/******************************************************* ADC *************************************************************/
void adc_init(void) {
	ADMUX = (1 << REFS0) | (1 << MUX2); // AVcc as ref (5 V), ADC4 (A4)
	ADCSRA = (1 << ADEN)               // enable ADC
	| (1 << ADIE)               // enable ADC interrupt
	| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler=128
	sei();                             // ensure global interrupts enabled
	ADCSRA |= (1 << ADSC);             // start first conversion
}

ISR(ADC_vect) {
	emg_samples[emg_index++] = ADC;
	if (emg_index >= BUFFER_SIZE) {
		emg_index = 0;
		emg_buffer_full = 1;
	}
	ADCSRA |= (1 << ADSC); // start next conversion
}

/************************************************ Timer1: 1 Hz interrupt for blinking *************************************************/
void timer1_init(void) {
	// Configure Timer1 in CTC mode (clear on compare match)
	TCCR1B |= (1 << WGM12);
	// Set prescaler to 256
	TCCR1B |= (1 << CS12);
	// Calculate OCR1A for 1 Hz tick:
	// 16,000,000 / 256 = 62500 counts per second
	// OCR1A = 62500 - 1
	OCR1A = 62499;
	// Enable Timer1 compare-match interrupt
	TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
	blink_flag = 1;
}

/************************************************ RMS Calculation ********************************************************/
uint16_t calculate_RMS(void) {
	uint32_t sum = 0;
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		sum += emg_samples[i];
	}
	uint16_t mean = sum / BUFFER_SIZE;

	uint32_t sum_squares = 0;
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		int16_t centered = (int16_t)emg_samples[i] - (int16_t)mean;
		sum_squares += (uint32_t)centered * centered;
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

/************************************************ Helper: generate new filename ******************************************/
// Scans "EMG000.TXT", "EMG001.TXT", … up to "EMG999.TXT"
// and returns the first one that does not yet exist.
// If all 000–999 are taken, returns "EMG999.TXT" as fallback.
static void get_new_filename(char *filename_out) {
	FILINFO fno;
	for (uint16_t idx = 0; idx < 1000; idx++) {
		// Format "EMG000.TXT" … "EMG999.TXT"
		sprintf(filename_out, "EMG%03u.TXT", idx);
		if (f_stat(filename_out, &fno) == FR_NO_FILE) {
			// This one doesn’t exist yet ? use it
			return;
		}
	}
	// If all 000..999 exist, just use 999:
	strcpy(filename_out, "EMG999.TXT");
}

/************************************************ Helper: log one RMS line ************************************************/
// Must only be called after f_open(&file, ...) has succeeded.
static void log_rms_to_sd(uint32_t rms_mv) {
	char line[16];
	UINT bytes_written;
	// Format "<number>\n", e.g. "1234\n"
	uint8_t len = sprintf(line, "%lu\n", rms_mv);
	f_write(&file, line, len, &bytes_written);
	// (Optional: verify bytes_written == len / check return code)
}

/************************************************ Screen A: live EMG visualization ***********************************************/
void ScreenA(void) {
	if (emg_buffer_full) {
		emg_buffer_full = 0;
		rms_adc = calculate_RMS();
		// Scale (×4) for display + USART:
		rms_mv = ((uint32_t)rms_adc * VREF * 4) / 1023;
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
				PORTB |= (1 << PB7);  // LED on
				closeHand();
				underThreshold = 0;
			}
		}
		if (rms_mv <= threshold) {
			underThreshold++;
			if (underThreshold == 5) {
				PORTB &= ~(1 << PB7); // LED off
				openHand();
				overThreshold = 0;
			}
		}
	}
}

/************************************************ Screen B: log EMG to SD & blink background ***********************************/
void ScreenB(void) {
	static uint8_t blink_state = 0;

	// Toggle background color when blink_flag is set (once per second)
	if (blink_flag) {
		blink_flag = 0;
		blink_state = !blink_state;
		if (blink_state) {
			// White background (RGB565 max)
			BackgroundColor(31, 63, 31);
			} else {
			// Black background
			BackgroundColor(0, 0, 0);
		}
	}

	if (emg_buffer_full) {
		emg_buffer_full = 0;
		rms_adc = calculate_RMS();
		rms_mv = ((uint32_t)rms_adc * VREF * 4) / 1023;
		log_rms_to_sd(rms_mv);
	}
}

/*********************************************** Main *******************************************************************/
int main(void) {
	// Initialize UART, ADC, PWM, Display, Touch, Pins
	USART0_Init(MYUBRR);
	adc_init();
	pwm_init();
	DisplayInit();
	InitTouchInterrupt();
	init_pins();
	CalibrateTouchScreen();
	DDRB |= (1 << PB7);  // Debug LED pin

	// Enable interrupts and start Timer1
	sei();
	timer1_init();

	current_state = STATE_SCREEN_A;
	x = 319;

	// Draw Screen A axes once at startup
	InitCoordinate();

	for (;;) {
		switch (current_state) {

			case STATE_SCREEN_A:
			// Run ScreenA() until a touch is detected
			while ( READ(D_IRQ_PINR, D_IRQ_PIN) ) {
				ScreenA();
			}
			// Finger-down ? debounce + wait for lift
			_delay_ms(50);
			while ( !READ(D_IRQ_PINR, D_IRQ_PIN) );
			_delay_ms(50);

			// Switch to Screen B
			current_state = STATE_SCREEN_B;
			// Ensure ScreenB background is set on first call
			{
				extern void ScreenB(void);
				// If "drawn_bg" logic existed, reset it here. In this version, Timer1 ISR will handle blinking.
			}
			// Initialize Screen B background immediately
			BackgroundColor(0, 0, 0);
			break;

			case STATE_SCREEN_B: {
				// Mount SD card
				if (f_mount(&fs, "", 1) != FR_OK) {
					// Mount failed ? hang
					while (1) { }
				}

				// Generate new filename like "EMG000.TXT"
				char fname[16];
				get_new_filename(fname);

				// Open/create the file for writing
				if (f_open(&file, fname, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
					// Open failed ? hang
					while (1) { }
				}

				// Continuously log data and blink until next touch
				while ( READ(D_IRQ_PINR, D_IRQ_PIN) ) {
					ScreenB();
				}

				// Touch detected ? close file, then go back to Screen A
				f_close(&file);

				_delay_ms(50);
				while ( !READ(D_IRQ_PINR, D_IRQ_PIN) );
				_delay_ms(50);

				// Return to Screen A
				current_state = STATE_SCREEN_A;
				InitCoordinate();
				x = 319;
				overThreshold  = 0;
				underThreshold = 0;
				break;
			}
		}
	}
}
