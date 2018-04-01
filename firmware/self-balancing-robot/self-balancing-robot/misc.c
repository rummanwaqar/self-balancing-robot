
/*
 * misc.c
 *
 * Created: 2018-04-01 5:12:15 PM
 *  Author: rumma
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>

#include "defines.h"
#include "misc.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void tick_timer(void)
{
	// enable timer2 for speed calculation
	TCCR2A |= (1<<WGM21);				// CTC mode
	TCCR2B |= (1<<CS22);				// ps=64;
	TIMSK2 |= (1<< OCIE2A);				// interrupt on compare match
	OCR2A = 229;						// 1 kHz
}

void gpio_init(void)
{
	// initialize LEDs as OFF
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}