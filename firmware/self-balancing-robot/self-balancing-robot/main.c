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
#include "i2cmaster.h"

FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

void gpio_init(void);
void init_encoders(void);
void tick_timer(void);
int8_t get_direction(uint8_t current, uint8_t prev);

volatile long enc1 = 0;
volatile long enc2 = 0;
volatile float speed1 = 0;
volatile float speed2 = 0;

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	i2c_init();
	init_encoders();
	tick_timer();
	
	sei();	

    while (1) 
    {
		printf("%d %d\n", (int)speed1, (int)speed2);
		
		//PORT(LED_PORT) ^= _BV(LED_RED);
		_delay_ms(200);
    }
}

void gpio_init(void)
{
	// initialize LEDs as OFF
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}

void init_encoders(void)
{
	// initialize encoder as inputs
	DDR(EN_1A_PORT) &= ~( _BV(EN_1A_PIN) );
	DDR(EN_1B_PORT) &= ~( _BV(EN_1B_PIN) );
	DDR(EN_2A_PORT) &= ~( _BV(EN_2A_PIN) );
	DDR(EN_2B_PORT) &= ~( _BV(EN_2B_PIN) );
	
	// enable external interrupts
	EICRA |= _BV(ISC10) | _BV(ISC00);	// interrupt on any logical change
	EIMSK |= _BV(INT1) | _BV(INT0);		// enable both interrupts
}

void tick_timer(void)
{
	// enable timer2 for speed calculation
	TCCR2A |= (1<<WGM21)|(1<<COM2A1);	// CTC mode
	TCCR2B |= (1<<CS22);				// ps=64; 
	TIMSK2 |= (1<< OCIE2A);
	OCR2A = 229;						// 1 kHz
}

ISR(INT0_vect)
{
	// state = AB
	uint8_t state_current = (READ_PIN(EN_1A_PORT,EN_1A_PIN) << 1) | READ_PIN(EN_1B_PORT,EN_1B_PIN);
	if(state_current == 0 || state_current == 3)	// CW
	{
		enc1++;
	}
	else if(state_current == 1 || state_current == 2)	// CCW
	{
		enc1--;
	}
}

ISR(INT1_vect)
{
	// state = AB
	uint8_t state_current = (READ_PIN(EN_2A_PORT,EN_2A_PIN) << 1) | READ_PIN(EN_2B_PORT,EN_2B_PIN);
	if(state_current == 0 || state_current == 3)	// CW
	{
		enc2++;
	}
	else if(state_current == 1 || state_current == 2)	// CCW
	{
		enc2--;
	}
}

ISR(TIMER2_COMPA_vect)
{
	static uint8_t count = 0;
	
	if(count == 50)
	{
		// calculate speeds every 50ms
		speed1 = (float)enc1 * 20.0 / 1400.0 * 60.0;
		speed2 = (float)enc2 * 20.0 / 1400.0 * 60.0;
		enc1 = enc2 = 0;
		count = 0;
	}
	else
	{
		count++;
	}
	
}