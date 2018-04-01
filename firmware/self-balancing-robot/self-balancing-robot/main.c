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
#include "motor.h"

FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

#define ADDR (0x68 << 1)
#define MPU6050_RA_WHO_AM_I         0x75


void gpio_init(void);

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	motor_init();
	
	sei();
	
	motor_set_speed(0,0);

	int speed1, speed2;
	long enc1, enc2;
    while (1)
    {
		motor_get_speed(&speed1, &speed2);
		motor_get_encoder(&enc1, &enc2);
		
		printf("%d %d %ld %ld\n", (int)speed1, (int)speed2, enc1, enc2);
		
		PORT(LED_PORT) ^= _BV(LED_RED);
		_delay_ms(200);
    }
}

void gpio_init(void)
{
	// initialize LEDs as OFF
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}
