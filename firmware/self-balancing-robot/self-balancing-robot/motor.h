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
 * Calculates motor speed
 @param freq: frequency at which this function is called
 */
void motor_calculate_speed(int freq);

/*
 * get motor speed
 * param motor1: memory space to store motor1 value
 * param motor2: memory space to store motor2 value
 * returns 1 if speed available, 0 otherwise
 */
char motor_get_speed(float* motor1, float* motor2);

/*
 * set motor speed (0-255) using pwm
 */
void motor_set_speed(int8_t motor1, int8_t motor2);

/*
 * get encoder count
 */
void motor_get_encoder(long* enc1, long* enc2);