/*
 * misc.h
 *
 * Created: 2018-03-31 8:27:16 PM
 *  Author: rumman
 */ 


#ifndef _SBR_MISC_H_
#define _SBR_MISC_H_

long map(long x, long in_min, long in_max, long out_min, long out_max);

void tick_timer(void);

void gpio_init(void);

Vector3_t toEulerAngle(const float q0, const float q1, const float q2, const float q3);

Command_t parseCommand(char* input_string, int* value);

void display(DispFlags_t* disp_flags, Imu_t* imu_data, float q0, float q1, float q2, float q3, Vector3_t* rpy, Motor_t* motor1, Motor_t* motor2 );

#endif /* MISC_H_ */