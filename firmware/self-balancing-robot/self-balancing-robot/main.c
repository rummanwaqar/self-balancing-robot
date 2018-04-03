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
#include "MadgwickAHRS.h"

FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

volatile uint8_t pid_flag = 0;
volatile uint8_t imu_flag = 0;

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	motor_init();
	//tick_timer();
	
	sei();
	
	//init mpu6050
	//mpu6050_init();
	_delay_ms(50);

	
	char temp[10];
	Imu* imu_data;
	float speed1, speed2;
	long enc1, enc2;
	char *input_string;
	int set_point1 = 0, set_point2 = 0;
	while (1)
	{
		if((imu_data = mpu6050_getData()) != 0)
		{
			// convert to quaternion
			MadgwickAHRSupdateIMU(imu_data->gyro.x, imu_data->gyro.y, imu_data->gyro.z,
									imu_data->accel.x, imu_data->accel.y, imu_data->accel.z);
			Vector3 rpy = toEulerAngle(q0, q1, q2, q3);
			
			dtostrf(imu_data->accel.x, 3, 2, temp); printf("%s", temp);
			dtostrf(imu_data->accel.y, 3, 2, temp); printf(",%s", temp);
			dtostrf(imu_data->accel.z, 3, 2, temp); printf(",%s", temp);
			dtostrf(imu_data->gyro.x, 3, 2, temp); printf(",%s", temp);
			dtostrf(imu_data->gyro.y, 3, 2, temp); printf(",%s", temp);
			dtostrf(imu_data->gyro.z, 3, 2, temp); printf(",%s", temp);
			// quaternions
			dtostrf(q0, 3, 2, temp); printf(",%s", temp);
			dtostrf(q1, 3, 2, temp); printf(",%s", temp);
			dtostrf(q2, 3, 2, temp); printf(",%s", temp);
			dtostrf(q3, 3, 2, temp); printf(",%s", temp);
			
			// rpy
			dtostrf(rpy.x, 3, 2, temp); printf(",%s", temp);
			dtostrf(rpy.y, 3, 2, temp); printf(",%s", temp);
			dtostrf(rpy.z, 3, 2, temp); printf(",%s\n", temp);
		} 
		
		if(motor_get_speed(&speed1, &speed2))
		{
			motor_get_encoder(&enc1, &enc2);
			dtostrf(speed1, 3, 2, temp); printf("%s", temp);
			dtostrf(speed2, 3, 2, temp); printf(",%s", temp);
			printf(",%ld,%ld\n", enc1, enc2);
		}
		
		// read commands
		if(uart_input_available())
		{
			int value;
			Command cmd;
			input_string = uart_get_string();
			cmd=parseCommand(input_string, &value);
			switch (cmd)
			{
				case CMD_M1:
					printf("=Motor1:%d\n", value);
					set_point1 = value;
					break;
				case CMD_M2:
					printf("=Motor2:%d\n", value);
					set_point2 = value;
					break;	
				default:
					break;				
			}
		}
		motor_set_speed(set_point1, set_point2);
		
	} // end of while loop
}

/*
 * Tick timer at 1ms
 */
ISR(TIMER2_COMPA_vect)
{
	static uint8_t count = 0;
	
	count++;
	
	if(count == 1000/(int)ENC_RATE)
	{
		motor_calculate_speed(ENC_RATE);
		count = 0;
		PORT(LED_PORT) ^= _BV(LED_RED);
	}
	
}
