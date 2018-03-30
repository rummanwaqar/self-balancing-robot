/*
 * self-balancing-robot.c
 *
 * Created: 2018-03-29 7:45:28 PM
 * Author : rumman
 */ 

#include "defines.h"

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>

void gpio_init(void);

int main(void)
{
	gpio_init();

    while (1) 
    {
		
		PORT(LED_PORT) ^= _BV(LED_RED);
		
		
		_delay_ms(1000);
    }
}

void gpio_init(void)
{
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}

