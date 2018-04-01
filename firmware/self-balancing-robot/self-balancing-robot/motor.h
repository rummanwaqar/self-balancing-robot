/*
 * motor.h
 *
 * Created: 2018-03-31 7:13:03 PM
 *  Author: rumman
 */ 

/*
 * Initialize motor and encoders
 */
void motor_init(void);

/*
 * get motor speed
 * param motor1: memory space to store motor1 value
 * param motor2: memory space to store motor2 value
 */
void motor_get_speed(int16_t* motor1, int16_t* motor2);

/*
 * set motor speed (0-100%) using pwm
 */
void motor_set_speed(int8_t motor1, int8_t motor2);

/*
 * get encoder count
 */
void motor_get_encoder(long* enc1, long* enc2);