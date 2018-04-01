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
#include "motor.h"

FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

void gpio_init(void);

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	motor_init();
	tick_timer();
	
	sei();
	
	motor_set_speed(0,0);

	int speed1, speed2;
	long enc1, enc2;
	
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

ISR(TIMER2_COMPA_vect)
{
	static uint8_t count = 0;
	
	if(count == 1000/(int)ENC_RATE)
	{
		motor_calculate_speed(ENC_RATE);
		count = 0;
	}
	else
	{
		count++;
	}
}
