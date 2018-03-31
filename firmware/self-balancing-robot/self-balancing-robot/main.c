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
#include "mpu6050.h"

FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

void gpio_init(void);

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	i2c_init();
	
	sei();
	
	//init mpu6050
	mpu6050_init();
	_delay_ms(50);
	
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	double axg = 0;
	double ayg = 0;
	double azg = 0;
	double gxds = 0;
	double gyds = 0;
	double gzds = 0;
	
    while (1) 
    {
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);

		printf("$%d %d %d;\n", ax,ay,az);

		_delay_ms(100);
		
		
		PORT(LED_PORT) ^= _BV(LED_RED);
		_delay_ms(100);
    }
}

void gpio_init(void)
{
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}

