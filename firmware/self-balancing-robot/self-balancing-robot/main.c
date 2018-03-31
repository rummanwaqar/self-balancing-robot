/*
 * self-balancing-robot.c
 *
 * Created: 2018-03-29 7:45:28 PM
 * Author : rumman
 */ 

#include "defines.h"

#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "uart.h"
#include "i2cmaster.h"

FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

#define ADDR (0x68 << 1)
#define MPU6050_RA_WHO_AM_I         0x75


void gpio_init(void);

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	i2c_init();
	
	sei();
	
	char buffer[10];
	unsigned char ret;

    while (1) 
    {
		if(i2c_start(ADDR | I2C_WRITE) == 0)
		{
			i2c_write(MPU6050_RA_WHO_AM_I);
			_delay_us(10);
			i2c_rep_start(ADDR | I2C_READ);
			ret = i2c_readNak();                    // read one byte from EEPROM
			i2c_stop();
			itoa(ret, buffer, 16);
			printf("Result: 0x%s\n", buffer);
		}
		else
		{
			printf("Failed");
		}
		
		
		
		PORT(LED_PORT) ^= _BV(LED_RED);
		_delay_ms(100);
    }
}

void gpio_init(void)
{
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}

