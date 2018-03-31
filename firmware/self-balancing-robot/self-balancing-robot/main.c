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
	
	sei();
	
	//init mpu6050
	mpu6050_init();
	_delay_ms(50);
	
	//mpu6050_dmpInitialize();
	//mpu6050_dmpEnable();
	_delay_ms(10);
	
	
	int ax, ay, az, gx, gy, gz;
	double axg, ayg, azg, gxds, gyds, gzds;
	double qw = 1.0f;
	double qx = 0.0f;
	double qy = 0.0f;
	double qz = 0.0f;
	double roll = 0.0f;
	double pitch = 0.0f;
	double yaw = 0.0f;
	
	char temp[10];
	
    while (1) 
    {
		//if(mpu6050_getQuaternionWait(&qw, &qx, &qy, &qz)) {
			//mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);
		//}
		
		// for MAHONY filter
		//mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
		//mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
		// for no filter
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		//mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		
		
		
		
		
		
		PORT(LED_PORT) ^= _BV(LED_RED);
		_delay_ms(100);
    }
}

void gpio_init(void)
{
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}

