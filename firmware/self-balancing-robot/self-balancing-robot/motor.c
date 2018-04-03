/*
 * motor.c
 *
 * Created: 2018-03-31 7:12:56 PM
 *  Author: rumman
 */ 

#include "defines.h"
#include <avr/io.h>
#include <util/atomic.h>
#include <stdlib.h>

#include "misc.h"
#include "motor.h"

// private functions 
void init_encoders(void);
void motor_timer_init(void);
void setup_pwm();

volatile long enc1 = 0;
volatile long enc2 = 0;
volatile long enc1_total = 0;
volatile long enc2_total = 0;
volatile float speed1 = 0;
volatile float speed2 = 0;
volatile float speed_flag = 0;

// convert encoders to RPM
#define ENC_CONST	1.0 / (ENC_MODE * ENC_COUNT_REV) * 60.0

const int QEM[16] = {0,1,-1,0,-1,0,0,+1,+1,0,0,-1,0,-1,+1,0}; // Quadrature Encoder Matrix

void motor_init(void)
{
	init_encoders();
	setup_pwm();
}

void init_encoders(void)
{
	// initialize encoder as inputs
	DDR(EN_1A_PORT) &= ~( _BV(EN_1A_PIN) );
	DDR(EN_1B_PORT) &= ~( _BV(EN_1B_PIN) );
	DDR(EN_2A_PORT) &= ~( _BV(EN_2A_PIN) );
	DDR(EN_2B_PORT) &= ~( _BV(EN_2B_PIN) );
	
	// Encoder 1 is connected to PCINT1..2
	PCICR |= _BV(PCIE0);					// Enable interrupt on PCINT[0..7]
	PCMSK0 |= _BV(PCINT1) | _BV(PCINT2);	// PCINT1 & PCINT2 enabled (PB1 and PB2)
	
	// Encoder 2 is connected to PCINT19..20
	PCICR |= _BV(PCIE2);					// Enable interrupt on PCINT[16..23]
	PCMSK2 |= _BV(PCINT19) | _BV(PCINT20);	// PCINT19 & PCINT20 enabled (PD3 and PD4)
}

/*
 * setup timers for motor speed PWM
 */
void setup_pwm(void)
{
	// set up timer0
	MOTOR_PWM1 = 0;
	MOTOR_PWM2 = 0;
	TCCR0A |= _BV(WGM00);						// Phase Correct PWM
	TCCR0A |= _BV(COM0A1) | _BV(COM0B1);		// non-inverting (on clear)
	TCCR0B |= _BV(CS00);						// start timer0 (28.9 kHz) PS=1
	
	// set up pwm pins
	DDR(MOTOR_PORT) |= (_BV(MOTOR_1_PIN) | _BV(MOTOR_2_PIN));
	// set up dir pins as output
	PORT(MOTOR_DIR1_PORT) &= ~(_BV(MOTOR_DIR1_PIN));
	PORT(MOTOR_DIR2_PORT) &= ~(_BV(MOTOR_DIR2_PIN));
	DDR(MOTOR_DIR1_PORT) |= (_BV(MOTOR_DIR1_PIN));
	DDR(MOTOR_DIR2_PORT) |= (_BV(MOTOR_DIR1_PIN));
	
}

void motor_set_speed(int8_t motor1, int8_t motor2)
{
	if(motor1 < 0)
	{
		PORT(MOTOR_DIR1_PORT) |= _BV(MOTOR_DIR1_PIN);
		MOTOR_PWM1 = -1 * motor1;
	}
	else
	{
		PORT(MOTOR_DIR1_PORT) &= ~(_BV(MOTOR_DIR1_PIN));
		MOTOR_PWM1 = motor1;
	}
	if(motor2 < 0)
	{
		PORT(MOTOR_DIR2_PORT) |= _BV(MOTOR_DIR2_PIN);
		MOTOR_PWM2 = -1 * motor2;
	}
	else
	{
		PORT(MOTOR_DIR2_PORT) &= ~(_BV(MOTOR_DIR2_PIN));
		MOTOR_PWM2 = motor2;
	}
}

char motor_get_speed(float* motor1, float* motor2)
{
	if(speed_flag)
	{
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			*motor1 = speed1;
			*motor2 = speed2;
			speed_flag = 0;
			return 1;
		}
	}
	return 0;
}

void motor_get_encoder(long* enc1, long* enc2)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		*enc1 = enc1_total;
		*enc2 = enc2_total;
	}
}

/*
 * Handles encoder 1 (Trigger on any edge)
 */
ISR(PCINT0_vect)
{
	static uint8_t state_prev = 0;
	// state = AB
	uint8_t state_current = (READ_PIN(EN_1A_PORT,EN_1A_PIN) << 1) | READ_PIN(EN_1B_PORT,EN_1B_PIN);
	enc1 += QEM[state_prev*4+state_current];
	state_prev = state_current;
}

/*
 * Handles encoder 2 (Trigger on any edge)
 */
ISR(PCINT2_vect)
{
	static uint8_t state_prev = 0;
	// state = AB
	uint8_t state_current = (READ_PIN(EN_2A_PORT,EN_2A_PIN) << 1) | READ_PIN(EN_2B_PORT,EN_2B_PIN);
	enc2 += QEM[state_prev*4+state_current];
	state_prev = state_current;
}

/*
 * Speed calculation
 */
void motor_calculate_speed(int freq)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		speed1 = (float)enc1 * freq * ENC_CONST;
		speed2 = (float)enc2 * freq * ENC_CONST;
		enc1_total += enc1;
		enc2_total += enc2;
		enc1 = enc2 = 0;
		speed_flag = 1;
	}
}