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

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	i2c_init();
	init_encoders();
	
	sei();
	
	uint8_t address = 0;
	
	// initialize encoder
	

    while (1) 
    {

		//i2c_rep_start((address << 1)+I2C_READ);       // set device address and read mode
		//unsigned char ret = i2c_readNak();                    // read one byte from EEPROM
		//i2c_stop();
		//if(ret != 0)
		//{
			//puts("Found something");
		//}
		//
		//
		//address++;
		//if(address > 127)
		//{
			//address = 0;
			//puts("nothing found");
		//}	
		
		//PORT(LED_PORT) ^= _BV(LED_RED);
		//_delay_ms(200);
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

ISR(INT0_vect)
{
	if( PIN(EN_1A_PORT) & _BV(EN_1A_PIN) )
	{
		PORT(LED_PORT) |= _BV(LED_BLUE);
	}
	else
	{
		PORT(LED_PORT) &= ~(_BV(LED_BLUE));
	}
	 
}

ISR(INT1_vect)
{
	if( PIN(EN_2A_PORT) & _BV(EN_2A_PIN) )
	{
		PORT(LED_PORT) |= _BV(LED_GREEN);
	}
	else
	{
		PORT(LED_PORT) &= ~(_BV(LED_GREEN));
	}
}