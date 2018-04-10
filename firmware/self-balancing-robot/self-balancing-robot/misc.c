
/*
 * misc.c
 *
 * Created: 2018-04-01 5:12:15 PM
 *  Author: rumma
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "defines.h"
#include "misc.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void tick_timer(void)
{
	// enable timer2 for speed calculation
	TCCR2A |= (1<<WGM21);				// CTC mode
	TCCR2B |= (1<<CS22);				// ps=64;
	TIMSK2 |= (1<< OCIE2A);				// interrupt on compare match
	OCR2A = 229;						// 1 kHz
}

void gpio_init(void)
{
	// initialize LEDs as OFF
	PORT(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
	DDR(LED_PORT) |= ( _BV(LED_RED) | _BV(LED_GREEN) | _BV(LED_BLUE) );
}

Vector3_t toEulerAngle(const float q0, const float q1, const float q2, const float q3)
{
	Vector3_t rpy;
	// roll (x-axis rotation)
	double sinr = +2.0 * (q0 * q1 + q2 * q3);
	double cosr = +1.0 - 2.0 * (q1 * q1 + q2 * q2);
	rpy.x = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q0 * q2 - q3 * q1);
	if (fabs(sinp) >= 1)
	rpy.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
	rpy.y = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q0 * q3 + q1 * q2);
	double cosy = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
	rpy.z = atan2(siny, cosy);
	
	return rpy;
}

Command_t parseCommand(char* input_string, int* value)
{
	char command[7];
	int temp_val;
	if(input_string[0] == '>')
	{
		if(sscanf(input_string+1, "%s %d", command, &temp_val ) )
		{
			if (strcmp(command, "m1") == 0)
			{
				if(temp_val <= MOTOR_MAX_RPM && temp_val >= MOTOR_MAX_RPM * -1)
				{
					*value = temp_val;
					return CMD_M1;
				}
			}
			else if (strcmp(command, "m2") == 0)
			{
				if(temp_val <= MOTOR_MAX_RPM && temp_val >= MOTOR_MAX_RPM * -1)
				{
					*value = temp_val;
					return CMD_M2;
				}
			}
			else if (strcmp(command, "m_p") == 0)
			{
				*value = temp_val;
				return CMD_M_P;
			}
			else if (strcmp(command, "m_i") == 0)
			{
				*value = temp_val;
				return CMD_M_I;
			}
			else if (strcmp(command, "m_d") == 0)
			{
				*value = temp_val;
				return CMD_M_D;
			}
			else if (strcmp(command, "d_imu") == 0)
			{
				if(temp_val == 0 || temp_val == 1)
				{
					*value = temp_val;
					return CMD_DISP_IMU;
				}
			}
			else if (strcmp(command, "d_quat") == 0)
			{
				if(temp_val == 0 || temp_val == 1)
				{
					*value = temp_val;
					return CMD_DISP_QUAT;
				}
			}
			else if (strcmp(command, "d_rpy") == 0)
			{
				if(temp_val == 0 || temp_val == 1)
				{
					*value = temp_val;
					return CMD_DISP_RPY;
				}
			}
			else if (strcmp(command, "d_mvel") == 0)
			{
				if(temp_val == 0 || temp_val == 1)
				{
					*value = temp_val;
					return CMD_DISP_MVEL;
				}
			}
			else if (strcmp(command, "d_mpid") == 0)
			{
				if(temp_val == 0 || temp_val == 1)
				{
					*value = temp_val;
					return CMD_DISP_MPID;
				}
			}
			else if (strcmp(command, "d_menc") == 0)
			{
				if(temp_val == 0 || temp_val == 1)
				{
					*value = temp_val;
					return CMD_DISP_MENC;
				}
			}
			else if (strcmp(command, "p_p") == 0)
			{
				*value = temp_val;
				return CMD_P_P;
			}
			else if (strcmp(command, "p_i") == 0)
			{
				*value = temp_val;
				return CMD_P_I;
			}
			else if (strcmp(command, "p_d") == 0)
			{
				*value = temp_val;
				return CMD_P_D;
			}
		}
	}
	return CMD_NULL;
}

void display(DispFlags_t* disp_flags, Imu_t* imu_data, float q0, float q1, float q2, float q3, Vector3_t* rpy, Motor_t* motor1, Motor_t* motor2 )
{
	char temp[10];
	if(disp_flags->imu_raw)
	{	// raw imu
		dtostrf(imu_data->accel.x, 2, 2, temp); printf("%s", temp);
		dtostrf(imu_data->accel.y, 2, 2, temp); printf(",%s", temp);
		dtostrf(imu_data->accel.z, 2, 2, temp); printf(",%s", temp);
		dtostrf(imu_data->gyro.x, 3, 2, temp); printf(",%s", temp);
		dtostrf(imu_data->gyro.y, 3, 2, temp); printf(",%s", temp);
		dtostrf(imu_data->gyro.z, 3, 2, temp); printf(",%s", temp);
	}
	if(disp_flags->imu_quat)
	{	// quaternions
		if(disp_flags->imu_raw) {
			dtostrf(q0, 2, 2, temp); printf(",%s", temp);
		}
		else
		{
			dtostrf(q0, 2, 2, temp); printf("%s", temp);
		}
		dtostrf(q1, 2, 2, temp); printf(",%s", temp);
		dtostrf(q2, 2, 2, temp); printf(",%s", temp);
		dtostrf(q3, 2, 2, temp); printf(",%s", temp);
	}
	if(disp_flags->imu_rpy)
	{	// rpy
		if(disp_flags->imu_quat || disp_flags->imu_raw)
		{
			dtostrf(rpy->x, 3, 2, temp); printf(",%s", temp);
		}
		else
		{
			dtostrf(rpy->x, 3, 2, temp); printf("%s", temp);
		}
		dtostrf(rpy->y, 3, 2, temp); printf(",%s", temp);
		dtostrf(rpy->z, 3, 2, temp); printf(",%s", temp);
	}
	if(disp_flags->motor_vel)
	{
		// motor velocity
		if(disp_flags->imu_quat || disp_flags->imu_raw || disp_flags->imu_rpy)
		{
			dtostrf(motor1->speed, 3, 1, temp); printf(",%s", temp);
		}
		else
		{
			dtostrf(motor1->speed, 3, 1, temp); printf("%s", temp);
		}
		dtostrf(motor2->speed, 3, 1, temp); printf(",%s", temp);
	}
	if(disp_flags->motor_pid)
	{
		// pid set point and effort
		if(disp_flags->imu_quat || disp_flags->imu_raw || disp_flags->imu_rpy || disp_flags->motor_vel)
		{
			printf(",%d,%d,%d,%d", motor1->set_point, motor1->effort, motor2->set_point, motor2->effort);
		}
		else
		{
			printf("%d,%d,%d,%d", motor1->set_point, motor1->effort, motor2->set_point, motor2->effort);
		}
	}
	if(disp_flags->motor_enc)
	{
		//motor encoder count
		if(disp_flags->imu_quat || disp_flags->imu_raw || disp_flags->imu_rpy || disp_flags->motor_vel || disp_flags->motor_pid)
		{
			printf(",%ld,%ld", motor1->enc, motor2->enc);
		}
		else
		{
			printf("%ld,%ld", motor1->enc, motor2->enc);
		}
	}
	if(disp_flags->imu_quat || disp_flags->imu_raw || disp_flags->imu_rpy || disp_flags->motor_enc || disp_flags->motor_pid || disp_flags->motor_pid || disp_flags->motor_vel)
	{
		printf("\n");
	}
}