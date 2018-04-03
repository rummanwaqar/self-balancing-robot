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
#include "mpu6050.h"
#include "motor.h"
#include "misc.h"

FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

volatile uint8_t pid_flag = 0;
volatile uint8_t imu_flag = 0;

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	motor_init();
	tick_timer();
	
	sei();
	
	motor_set_speed(0,0);

	//int speed1, speed2;
	//long enc1, enc2;
	
	//init mpu6050
	mpu6050_init();
	_delay_ms(50);

	
	char temp[10];
	Imu* imu_data;
	while (1)
	{
		if((imu_data = mpu6050_getData()) != 0)
		{
			dtostrf(imu_data->accel.x, 3, 2, temp); printf("%s", temp);
			dtostrf(imu_data->accel.y, 3, 2, temp); printf(",%s", temp);
			dtostrf(imu_data->accel.z, 3, 2, temp); printf(",%s", temp);
			dtostrf(imu_data->gyro.x, 3, 2, temp); printf(",%s", temp);
			dtostrf(imu_data->gyro.y, 3, 2, temp); printf(",%s", temp);
			dtostrf(imu_data->gyro.z, 3, 2, temp); printf(",%s\n", temp);
		} 

		//motor_get_speed(&speed1, &speed2);
		//motor_get_encoder(&enc1, &enc2);
		//if(pid_flag)
		//{
			//dtostrf(pitch, 3, 2, temp);
			//printf("%s", temp);
			//dtostrf(roll, 3, 2, temp);
			//printf("\t%s", temp);
			//dtostrf(yaw, 3, 2, temp);
			//printf("\t%s\n", temp);
			//printf("%d %d %d\n", ax, ay, az);
			//pid_flag = 0;
			//
		//}
		//printf("%d %d %ld %ld\n", (int)speed1, (int)speed2, enc1, enc2);
		
		//PORT(LED_PORT) ^= _BV(LED_RED);
		//_delay_ms(200);
	}
}

/*
 * Tick timer at 1ms
 */
ISR(TIMER2_COMPA_vect)
{
	static uint8_t count = 0;
	static uint8_t pidCount = 0;
	
	count++;
	pidCount++;
	
	if(count == 1000/(int)ENC_RATE)
	{
		motor_calculate_speed(ENC_RATE);
		count = 0;
	}
	
	if(pidCount == 1000/(int)PID_RATE)
	{
		pid_flag = 1;
		pidCount = 0;
	}
	
}
