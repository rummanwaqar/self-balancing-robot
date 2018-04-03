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
#include "pid.h"

FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

volatile char disp_update_flag = 0;

int main(void)
{
	stdout = &uart_stream;
	gpio_init();
	uart_init();
	motor_init();
	tick_timer();
	mpu6050_init();
	_delay_ms(50);
	
	sei();
	
	// display settings
	DispFlags_t disp_flags = {.imu_raw=0, .imu_quat=0, .imu_rpy=1, .motor_vel=1, .motor_pid=0, .motor_enc=0};
	
	// initialize motors with pids
	Motor_t motor1, motor2;
	motor1.set_point = motor2.set_point = 0;
	float m_p=PID_M_P, m_i=PID_M_I, m_d=PID_M_D;
	pid_init(m_p, m_i, m_d, PID_M_I_WINDUP, MOTOR_MAX_EFFORT, &motor1.pidData);
	pid_init(m_p, m_i, m_d, PID_M_I_WINDUP, MOTOR_MAX_EFFORT, &motor2.pidData);
	
	Imu_t* imu_data;	// raw imu data
	Vector3_t rpy;		// roll pitch yaw information

	while (1)
	{
		if((imu_data = mpu6050_getData()) != 0)
		{
			// convert to quaternion
			MadgwickAHRSupdateIMU(imu_data->gyro.x, imu_data->gyro.y, imu_data->gyro.z,
									imu_data->accel.x, imu_data->accel.y, imu_data->accel.z);
			rpy = toEulerAngle(q0, q1, q2, q3);	// Madgwick library has globals q0..q3 for quaternion pose
			
			if(fabs(rpy.y) < 0.0872665)
			{
				PORT(LED_PORT) &= ~(_BV(LED_GREEN));
				PORT(LED_PORT) |= _BV(LED_BLUE);
				PORT(LED_PORT) |= _BV(LED_RED);
			}
			else if(fabs(rpy.y) < 0.174533)
			{
				PORT(LED_PORT) |= (_BV(LED_GREEN));
				PORT(LED_PORT) &= ~(_BV(LED_BLUE));
				PORT(LED_PORT) |= _BV(LED_RED);
			}
			else
			{
				PORT(LED_PORT) |= (_BV(LED_GREEN));
				PORT(LED_PORT) |= _BV(LED_BLUE);
				PORT(LED_PORT) &= ~(_BV(LED_RED));
			}
		} 
		
		if(motor_get_speed(&motor1.speed, &motor2.speed))
		{
			motor_get_encoder(&motor1.enc, &motor2.enc);
			// motor pid
			motor1.effort = (int)pid_controller(motor1.set_point, motor1.speed, &motor1.pidData);
			motor2.effort = (int)pid_controller(motor2.set_point, motor2.speed, &motor2.pidData);
			// motor effort
			if(abs(motor1.effort) < MOTOR_MIN_EFFORT) motor1.effort = 0;
			if(abs(motor2.effort) < MOTOR_MIN_EFFORT) motor2.effort = 0;
			motor_set_speed(motor1.effort, motor2.effort);
		}
		
		// display uart
		if(disp_update_flag)
		{
			display(&disp_flags, imu_data, q0, q1, q2, q3, &rpy, &motor1, &motor2);
			disp_update_flag = 0;
		}
		
		// read commands
		if(uart_input_available())
		{
			int value;
			Command_t cmd=parseCommand(uart_get_string(), &value);
			switch (cmd)
			{
				case CMD_M1:
					printf("=Motor1:%d\n", value);
					motor1.set_point = value;
					break;
				case CMD_M2:
					printf("=Motor2:%d\n", value);
					motor2.set_point = value;
					break;
				case CMD_M_P:
					printf("=Motor P(x100):%d\n", value);
					m_p = (float)value/100.0;
					motor2.pidData.P_Factor = motor1.pidData.P_Factor = m_p;
					pid_reset(&motor1.pidData); pid_reset(&motor2.pidData);
					break;
				case CMD_M_I:
					printf("=Motor I(x100):%d\n", value);
					m_i = (float)value/100.0;
					motor2.pidData.I_Factor = motor1.pidData.I_Factor = m_i;
					pid_reset(&motor1.pidData); pid_reset(&motor2.pidData);
					break;
				case CMD_M_D:
					printf("=Motor D(x100):%d\n", value);
					m_d = (float)value/100.0;
					motor2.pidData.D_Factor = motor1.pidData.D_Factor = m_d;
					pid_reset(&motor1.pidData); pid_reset(&motor2.pidData);
					break;
				case CMD_DISP_IMU:
					printf("=Display IMU:%d\n", value);
					disp_flags.imu_raw = value;
					break;
				case CMD_DISP_QUAT:
					printf("=Display Quat:%d\n", value);
					disp_flags.imu_quat= value;
					break;
				case CMD_DISP_RPY:
					printf("=Display RPY:%d\n", value);
					disp_flags.imu_rpy = value;
					break;
				case CMD_DISP_MVEL:
					printf("=Display motor vel:%d\n", value);
					disp_flags.motor_vel = value;
					break;
				case CMD_DISP_MPID:
					printf("=Display motor PID:%d\n", value);
					disp_flags.motor_pid = value;
					break;
				case CMD_DISP_MENC:
					printf("=Display motor enc:%d\n", value);
					disp_flags.motor_enc = value;
					break;
				default:
					break;		
			}
		}		
	} // end of while loop
}

/*
 * Tick timer at 1ms
 */
ISR(TIMER2_COMPA_vect)
{
	static uint8_t motor_count = 0;
	static uint8_t display_count = 0;
	if(display_count == 1000/DISP_RATE)
	{
		disp_update_flag = 1;
	}
	if(motor_count == 1000/(int)ENC_RATE)
	{
		motor_calculate_speed(ENC_RATE);
		motor_count = 0;
	}
	motor_count++;
	display_count++;
}
