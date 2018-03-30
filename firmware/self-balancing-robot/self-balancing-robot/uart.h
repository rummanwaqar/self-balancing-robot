/*
 * @file: uart.h
 * @author: rumman waqar
 * UART Library
 * Note: putchar will automatically add a \r before each \n
 */

#ifndef _SBR_UART_H_
#define _SBR_UART_H_

#include "defines.h"

#include <stdio.h>
#include <avr/io.h>

#define BAUD_PRESCALE	(char)(((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

/*
 * initializes uart with 8bit 1 stop bit size, BAUD_RATE set by BAUD_PRESCALE define and enables receive interrupt
 */
void uart_init(void);

/*
 * outputs a single character on a stream using UART0
 */
int uart_putchar(char c,  FILE *stream);

/*
 * copies input from the temp array to the processing array 
 * resets the input ready flag
 * returns pointer to buffer
 */
char* uart_get_string();

/*
 * resets the temp input array and the input ready flag
 */
void uart_flush_input();

/*
 * checks if input available
 */
int uart_input_available(void);

#endif