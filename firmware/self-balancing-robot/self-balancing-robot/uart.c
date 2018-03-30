/*
 * @file: uart.c
 * @author: rumman waqar
 * Implementation of Uart
 * Check uart.h for documentation
 */

#include "uart.h"

#include <string.h>
#include <util/atomic.h>

// variables to handle input from UART
volatile char temp_input[UART_MAX_BUFFER_SIZE];	// temp  uart input
char input_buffer[UART_MAX_BUFFER_SIZE];			// input array to process data
volatile unsigned int data_count;			// write index in temp input array
volatile unsigned char input_ready;			// flag to check if an input line has been received

void uart_init(void)
{
	// setup uart
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes
	UBRR0H = (BAUD_PRESCALE >> 8);
	UBRR0L = BAUD_PRESCALE;
	// Enable the USART Receive interrupt
	UCSR0B |= (1 << RXCIE0 );
}

int uart_putchar(char c,  FILE *stream) {
	if (c == '\n') {
		uart_putchar('\r', stream);
	}
	while ((UCSR0A & (1 << UDRE0)) == 0) {}; // wait until register is empty
	UDR0 = c;
	return 0;
}

char* uart_get_string()
{
	ATOMIC_BLOCK(ATOMIC_FORCEON) 
	{
		// copy input 
		strcpy(input_buffer, (const char*) temp_input);
		// reset input
		temp_input[0] = '\0';
		input_ready = 0;
	}
	return input_buffer;
}

void uart_flush_input()
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		// reset input
		temp_input[0] = '\0';
		input_ready = 0;
	}
}

int uart_input_available(void)
{
	return input_ready;
}

ISR (USART_RX_vect)
{
	// Get data from the USART in register
	char recChar = UDR0;
	PORTC ^= _BV(PORTC2);
	
	// End of line!
	if (recChar == '\n' || recChar == '\r') 
	{
		if(data_count > 0)
		{
			temp_input[data_count] = '\0';
			input_ready = 1;
			data_count = 0;
		}
	}
	else
	{
		temp_input[data_count++] = recChar;
	}
	
	if(data_count >= UART_MAX_BUFFER_SIZE)
	{
		data_count = 0;
	}
}