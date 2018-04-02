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
	
	double acc_total_vector;
	double angle_pitch_acc;
	double angle_roll_acc;
	double angle_yaw_acc;
	
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
		if(imu_flag) // IMU_RATE
		{
			mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
			mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
			yaw += gzds / IMU_RATE;
			
			pitch += gxds / IMU_RATE;	//Calculate the traveled pitch angle and add this to the angle_pitch variable
			roll += gyds / IMU_RATE;
			
			//float temp = pitch + roll * sin(yaw * 3.142/180.0);	
			//roll -= pitch * sin(yaw * 3.142/180.0);	//If the IMU has yawed transfer the pitch angle to the roll angel
			//pitch = temp;
			acc_total_vector = sqrt((axg*axg)+(ayg*ayg)+(azg*azg));
			angle_pitch_acc = asin(ayg/acc_total_vector) * 180.0/3.142;
			angle_roll_acc = asin(axg/acc_total_vector) * -180.0/3.142;
			
			// kalman filter
			pitch = pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
			roll = roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
			
			PORT(LED_PORT) ^= _BV(LED_RED);
			imu_flag = 0;
		}
		
		//motor_get_speed(&speed1, &speed2);
		//motor_get_encoder(&enc1, &enc2);
		if(pid_flag)
		{
			dtostrf(pitch, 3, 2, temp);
			printf("%s", temp);
			dtostrf(roll, 3, 2, temp);
			printf("\t%s", temp);
			dtostrf(yaw, 3, 2, temp);
			printf("\t%s\n", temp);
			//printf("%d %d %d\n", ax, ay, az);
			pid_flag = 0;
			
		}
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
	static uint8_t imuCount = 0;
	
	count++;
	pidCount++;
	imuCount++;
	
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
	
	if(imuCount == 1000/(int)IMU_RATE)
	{
		imu_flag = 1;
		imuCount = 0;
	}
}
