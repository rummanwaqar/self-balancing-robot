/*
 * self-balancing-robot.c
 *
 * Created: 2018-03-29 7:45:28 PM
 * Author : rumman
 */ 

#include "defines.h"

#include <stdio.h>

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "uart.h"

FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

void gpio_init(void);

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	
	sei();

    while (1) 
    {
		puts("Hello world");
		
		PORT(LED_PORT) ^= _BV(LED_RED);
		
		
		_delay_ms(1000);
    }
}

void gpio_init(void)
{
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}

