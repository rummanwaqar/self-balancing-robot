/*
 * motor.h
 *
 * Created: 2018-03-31 7:13:03 PM
 *  Author: rumma
 */ 

void motor_init(void);
void motor_get_speed(int16_t* motor1, int16_t* motor2);
void motor_set_speed(int8_t motor1, int8_t motor2);