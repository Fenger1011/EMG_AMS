#define F_CPU 16000000UL	// CPU frequency

#include <avr/io.h>			// AVR I/O Header
#include <util/delay.h>		// Delays
#include <stdlib.h>			// For using itoa (int to ASCII conversion)
#include <math.h>			// For using sqrt()
#include <stdint.h>			// For fixed-width int types (uint16_t etc.)
#include <avr/interrupt.h>	// ISR() and sei()
#include <string.h>			// string manipulation

#include "USART_Driver.h"	// USART_Driver for debugging
#include "TFT_driver.h"		// TFT driver 
#include "XPT2046_Driver.h"	// XPT2046 Driver
#include "ff.h"				// FatFS library header (used to read/write to SD cards f_open(), f_write(), f_close())
#include "diskio.h"			// disk I/O used by FatFS (connects FatFS engine to SD driver)
#include "SD_Driver.h"		// SD card driver

FATFS fs;
FIL file;
UINT bw;

#define BAUD         9600				// Baud rate for UART
#define MYUBRR       (F_CPU/16/BAUD - 1)// Calculates baud rate for UART for baud rate register 

#define VREF         5000				// ADC reference voltage
#define BUFFER_SIZE  480				// EMG sample buffer size per processing window

// EMG buffer and flags
volatile uint16_t emg_samples[BUFFER_SIZE];	// EMG sample buffer of size BUFFER_SIZE = 480 (volatile because ISR updates this buffer)
volatile uint16_t emg_index       = 0;		// For knowing index in emg_samples[] array
volatile uint8_t  emg_buffer_full = 0;		// Flag for when buffer is full and ready for processing / saving to SD etc.
volatile uint8_t blink_flag = 0;			// Blink flag (set in Timer1 interrupt)
extern volatile uint8_t touch_triggered;	// Touch flag for when touch triggered (defined in XPT2046_driver.c --> therefore extern volatile)

// EMG processing variables
uint16_t x            = 319;	// The leftmost position on the horizontal position on the TFT
uint16_t rms_adc      = 0;		// holds RMS value of EMG from ADC
uint32_t rms_mv       = 0;		// Holds RMS value in mV
uint16_t threshold    = 100;	// EMG signal activation threshold for motor control
uint16_t overThreshold  = 0;	// Counter for consecutive 'windows' where the EMG signals are over threshold
uint16_t underThreshold = 0;	// Counter for consecutive 'windows' where the EMG signals are under threshold
char buffer[12];				// Used for converting numerical values into string for UART


// Screen states
typedef enum {
	STATE_SCREEN_A,	// Live EMG visualization and motor control screen
	STATE_SCREEN_B	// EMG data logging screen
} ScreenState;

// Tracks current screen state
ScreenState current_state = STATE_SCREEN_A;




/******************************************************* PWM *************************************************************/
// Initializes PWM on pin PH5 using Timer4 on Channel C
void pwm_init(void) {
	DDRH |= (1 << PH5); // PH5 as output (OC4C)
	
	TCCR4A = (1 << COM4C1) | (1 << WGM41);  // Non-inverting, Fast PWM
	TCCR4B = (1 << WGM43) | (1 << WGM42)    // Fast PWM mode 14, ICR4 = TOP
	| (1 << CS41) | (1 << CS40);     // Prescaler = 64 (16MHz / 64 = 250 kHz)
	
	ICR4 = 4999; // 20ms period (50Hz): (16e6 / 64) / 50Hz = 5000 ? -1 = 4999
}


// Sets PWM duty cycle as a percentage
void pwm_set_duty(uint16_t duty_percent) {
	OCR4C = ((uint32_t)ICR4 * duty_percent) / 100;
}
/*************************************************************************************************************************/


/******************************************************* ADC *************************************************************/
// Initializes the ADC to read from channel ADC4 with AVcc as the reference
void adc_init(void) {
	ADMUX = (1 << REFS0) | (1 << MUX2);				// AVcc as reference, ADC4 as input channel
	ADCSRA = (1 << ADEN)							// Enable ADC
	| (1 << ADIE)									// Enable ADC interrupt
	| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// Prescaler=128 => f_ADC = 125kHz            ** 125kHz / 13 = 9.6kHz sampling **
	sei();											// Enable global interrupts
	ADCSRA |= (1 << ADSC);							// Start first conversion
}

// ISR triggers when ADC conversion complete
ISR(ADC_vect) {
	emg_samples[emg_index++] = ADC;	// Store ADC result in EMG sample buffer, increment index
	if (emg_index >= BUFFER_SIZE) {	// If buffer is full:
		emg_index = 0;					// Reset index
		emg_buffer_full = 1;			// Set flag that buffer is full
	}
	ADCSRA |= (1 << ADSC);			// start next conversion
}
/*************************************************************************************************************************/


/****************************************************** Timer1 ***********************************************************/
// Initializes Timer1 to generate 1Hz interrupt
void timer1_init(void) {
	TCCR1B |= (1 << WGM12);		// Config Timer1 in CTC mode 
	TCCR1B |= (1 << CS12);		// Prescaler=256 => (f_CPU / 256)
	OCR1A = 62499;				// Sets interrupt frequency at 1 Hz [(16MHz / 256) - 1]
	TIMSK1 |= (1 << OCIE1A);	// Enable Timer1 
}

// ISR for Timer1
ISR(TIMER1_COMPA_vect) {
	blink_flag = 1;				// Sets flag to signal 1 second has passed.
}
/*************************************************************************************************************************/


/************************************************ RMS Calculation ********************************************************/
// Calculates the RMS value of samples i buffer
uint16_t calculate_RMS(void) {
	
	// 1. Compute mean of all samples
	uint32_t sum = 0;
											
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		sum += emg_samples[i];			// Sum all samples
	}
	uint16_t mean = sum / BUFFER_SIZE;	// Calculate the mean

	
	// 2. Calculate sum of squared deviations from mean    \sum{x_i - \mu}^2
	uint32_t sum_squares = 0;
	
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		int16_t centered = (int16_t)emg_samples[i] - (int16_t)mean;	// Center sample around mean because of DC bias --> Cast to int16_t for negative values (some might be below the mean => negative numbers!)
		sum_squares += (uint32_t)centered * centered;				// Add square of sampled value
	}
	
	// 3. Calculate mean of squared differences    1/N * \sum{x_i - \mu}^2
	uint32_t mean_square = sum_squares / BUFFER_SIZE;
	
	// 4. Return the square root of the mean square	(RMS :))
	return (uint16_t)sqrt((double)mean_square);		// Cast to double for precision in sqrt operation --> final cast to uint16_t which is being returned
}
/*************************************************************************************************************************/


/************************************************ Motor Control **********************************************************/
// 'Closes' the servo motor
void closeHand(void) {
	pwm_set_duty(6);	
	_delay_ms(500);
	pwm_set_duty(0);
}

// 'Opens' the servo motor
void openHand(void) {
	pwm_set_duty(9);
	_delay_ms(475);
	pwm_set_duty(0);
}
/*************************************************************************************************************************/


/************************************************ Helpers for SD *******************************************************/
// Generates the correct filename for the EMG data
// Scans for available filenames in the format "EMG000.TXT" to "EMG999.TXT"
// Returns the first unused filename in 'filename_out'
// If all names are taken, defaults to "EMG999.TXT"
static void get_new_filename(char *filename_out) {
	FILINFO fno;	// File info struct used by FatFs to hold file metadata --> Used by f_stat
	
	for (uint16_t idx = 0; idx < 1000; idx++) {
		sprintf(filename_out, "EMG%03u.TXT", idx);			// Format index into a filename: "EMG000.TXT", "EMG001.TXT", ..., "EMG999.TXT"
		if (f_stat(filename_out, &fno) == FR_NO_FILE) {		// Check if file does NOT exist on SD card and then returns TRUE
			return;											// If file does not exist, stop loop and now filename_out contains the next file name
		}
	}

	strcpy(filename_out, "EMG999.TXT");						// If no filename available, use EMG999.txt
}


// Logs a single RMS value (in millivolts) to the open SD file.
// Must only be called after f_open(&file, ...) has succeeded.
static void log_rms_to_sd(uint32_t rms_mv) {
	char line[16];									// Buffer for holding text value of voltage
	UINT bytes_written;								// Variable to store number of bytes actually written
	uint8_t len = sprintf(line, "%lu\n", rms_mv);	// Format the RMS value followed by a newline, e.g., "1234\n"
	f_write(&file, line, len, &bytes_written);		// Write the formatted string to the SD card file
}
/*************************************************************************************************************************/


/************************************************ Screen A: live EMG visualization ***********************************************/
// Handles live EMG data processing, visualization, and motor/LED control
void ScreenA(void) {
	if (emg_buffer_full) {
		emg_buffer_full = 0;	// Reset the buffer-full flag	
		
		// Calculate RMS from buffer
		rms_adc = calculate_RMS();
		
		// Convert RMS to milivolts and scale by 4 (Gives better view on TFT)
		rms_mv = ((uint32_t)rms_adc * VREF * 4) / 1023;
		
		//*** Send result over UART (FOR DEBUGGING) ***//
		//itoa(rms_mv, buffer, 10);
		//USART0_SendString(buffer);
		//USART0_Transmit('\n');
		/***********************************************/
		
		// Map EMG to screen size
		uint16_t mapped_sample = ((rms_mv * 239UL) / 2000);
		
		// Draw EMG on screen at current x
		DrawEMG(mapped_sample, x);
		
		// Move x for scrolling effect
		x -= 3;
		
		// If x at end, reset to start of screen						
		if (x <= 1) {
			x = 319;			
			InitCoordinate();	// Resets the screen
		}
		
		// If RMS over threshold
		if (rms_mv >= threshold) {
			overThreshold++;			// Increment over threshold counter (helps smoothing)
			if (overThreshold == 3) {	// If over threshold for three increments
				//PORTB |= (1 << PB7);	// LED on (FOR DEBUGGING)
				closeHand();			// Close the prosthesis (servo motor)
				underThreshold = 0;		// Reset counter
			}
		}
		
		// If RMS under threshold
		if (rms_mv <= threshold) {
			underThreshold++;			// Increment under threshold counter (helps smoothing)
			if (underThreshold == 5) {	// If over threshold for five increments (Stops the motor from flickering)
				//PORTB &= ~(1 << PB7);	// LED off (FOR DEBUGGING)
				openHand();				// Open the prosthesis (servo motor)
				overThreshold = 0;		// Reset counter
			}
		}
	}
}
/*************************************************************************************************************************/


/************************************************ Screen B: log EMG to SD & blink background ***********************************/
// Logs EMG data and blinks screen
void ScreenB(void) {
	static uint8_t blink_state = 0;	// Keeps track of blinking

	// Toggle background color when blink_flag is set (once per second)
	if (blink_flag) {
		blink_flag = 0;				// Reset flag
		blink_state = !blink_state;	// Flip the state
		
		if (blink_state) {
			BackgroundColor(31, 63, 31);	// Set background to white
			} else {
			BackgroundColor(0, 0, 0);		// Set background to black
		}
	}

	// If a new EMG buffer is full (from ISR)
	if (emg_buffer_full) {
		emg_buffer_full = 0;							// Reset flag
		rms_adc = calculate_RMS();						// Calculate RMS
		rms_mv = ((uint32_t)rms_adc * VREF * 4) / 1023;	// Convert RMS to militvolts 
		log_rms_to_sd(rms_mv);							// Log mV_RMS to SD card
	}
}
/*************************************************************************************************************************/


/*********************************************** Main *******************************************************************/
int main(void) {
	// Initialize peripherals
	//USART0_Init(MYUBRR);		// Initialize UART (FOR DEBUGGING)	
	adc_init();					// Initialize ADC
	pwm_init();					// Initialize PWM
	DisplayInit();				// Initialize TFT display
	InitTouchInterrupt();		// Initialize TFT touch screen
	init_pins();				// Initialize custom pins for bitbanged SPI (for XPT)
	CalibrateTouchScreen();		// Run calibrate touch screen as the first in loop -> Makes sure all future touch inputs are correct.
	//DDRB |= (1 << PB7);		// LED pin (FOR DEBUGGING)

	sei();							// Enable global timer interrupts
	timer1_init();					// Start Timer1 for 1 Hz interrupts (for screenB)

	current_state = STATE_SCREEN_A;	// Start in screen A (EMG visualization)
	x = 319;						// Set initial X coordinate for plotting

	InitCoordinate();				// Draw Screen A axes once at startup

	for (;;) {
		switch (current_state) {

			case STATE_SCREEN_A:
			// Continuously run Screen A (live view) until a touch is detected
			while ( READ(D_IRQ_PINR, D_IRQ_PIN) ) {
				ScreenA();
			}
			
			// Touch detected: debounce + wait for lift
			_delay_ms(50);
			while ( !READ(D_IRQ_PINR, D_IRQ_PIN) );
			_delay_ms(50);

			// Switch to Screen B (logging mode)
			current_state = STATE_SCREEN_B;
			
			// Initialize Screen B background immediately to black
			BackgroundColor(0, 0, 0);
			break;

			case STATE_SCREEN_B: {
				// Try to mount the SD card file system
				if (f_mount(&fs, "", 1) != FR_OK) {
					// Mount failed: enter infinite loop (system halt)
					while (1) { }
				}

				// Generate new unique filename on SD-card
				char fname[16];
				get_new_filename(fname);

				// Try to open/create the file for writing (overwrite if it exists)
				if (f_open(&file, fname, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
					// File open failed: halt
					while (1) { }
				}
				
				// Mounted the SD card file system and found unique file name, now ScreenB can run continuously.
				// Run Screen B logic (logging + background blinking) until touch
				while ( READ(D_IRQ_PINR, D_IRQ_PIN) ) {
					ScreenB();
				}

				// Touch detected: close file and return to Screen A
				f_close(&file);
				
				// Debounce touch
				_delay_ms(50);
				while ( !READ(D_IRQ_PINR, D_IRQ_PIN) );
				_delay_ms(50);
				
				// Reinitialize for Screen A view
				current_state = STATE_SCREEN_A;
				InitCoordinate();		// Redraw axis
				x = 319;				// Reset x position for plotting
				overThreshold  = 0;		// Reset flag
				underThreshold = 0;		// Reset flag
				break;
			}
		}
	}
}
