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

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	i2c_init();
	
	sei();
	
	uint8_t address = 0;
	
	// initialize encoder
	DDRD &= ~( _BV(PORTD2) );

    while (1) 
    {
		if(PIND & _BV(2))
		{
			PORT(LED_PORT) |= _BV(LED_GREEN);
		}
		else
		{
			PORT(LED_PORT) &= ~(_BV(LED_GREEN));
		}
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
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}

