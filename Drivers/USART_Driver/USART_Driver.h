#ifndef USART0_DRIVER_H_
#define USART0_DRIVER_H_

#include <avr/io.h>

// Initialize USART0 with given UBRR value (calculated externally)
void USART0_Init(unsigned int ubrr);

// Transmit single byte over USART0
void USART0_Transmit(unsigned char data);

// Transmit null-terminated string over USART0
void USART0_SendString(const char *str);

#endif /* USART0_DRIVER_H_ */
