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
volatile float speed1 = 0;
volatile float speed2 = 0;

// convert encoders to RPM
#define ENC_CONST	(ENC_RATE) / (ENC_MODE * ENC_COUNT_REV) * 60.0

void motor_init(void)
{
	init_encoders();
	motor_timer_init();
	setup_pwm();
}

void init_encoders(void)
{
	// initialize encoder as inputs
	DDR(EN_1A_PORT) &= ~( _BV(EN_1A_PIN) );
	DDR(EN_1B_PORT) &= ~( _BV(EN_1B_PIN) );
	DDR(EN_2A_PORT) &= ~( _BV(EN_2A_PIN) );
	DDR(EN_2B_PORT) &= ~( _BV(EN_2B_PIN) );
	
	// enable external interrupts
	EICRA |= _BV(ISC10) | _BV(ISC00);	// interrupt on any logical change
	EIMSK |= _BV(INT1) | _BV(INT0);		// enable both interrupts
}

/*
 * setup timers for motor speed PWM
 */
void setup_pwm(void)
{
	// set up timer0
	MOTOR_1 = 0;
	MOTOR_2 = 0;
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

void motor_timer_init(void)
{
	// enable timer2 for speed calculation
	TCCR2A |= (1<<WGM21);				// CTC mode
	TCCR2B |= (1<<CS22);				// ps=64;
	TIMSK2 |= (1<< OCIE2A);				// interrupt on compare match
	OCR2A = 229;						// 1 kHz
}

void motor_set_speed(int8_t motor1, int8_t motor2)
{
	if(motor1 != 0)
	{
		MOTOR_1 = map(abs(motor1), 0, 100, MOTOR_MIN_PWM, 255);
	}
	else
	{
		MOTOR_1 = 0;
	}
	if(motor2 != 0)
	{
		MOTOR_2 = map(abs(motor2), 0, 100, MOTOR_MIN_PWM, 255);
	}
	else
	{
		MOTOR_2 = 0;
	}
	
	if(motor1 < 0)
	{
		PORT(MOTOR_DIR1_PORT) |= _BV(MOTOR_DIR1_PIN);
	}
	else
	{
		PORT(MOTOR_DIR1_PORT) &= ~(_BV(MOTOR_DIR1_PIN));
	}
	if(motor2 < 0)
	{
		PORT(MOTOR_DIR2_PORT) |= _BV(MOTOR_DIR2_PIN);
	}
	else
	{
		PORT(MOTOR_DIR2_PORT) &= ~(_BV(MOTOR_DIR2_PIN));
	}
}

void motor_get_speed(int16_t* motor1, int16_t* motor2)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		*motor1 = speed1;
		*motor2 = speed2;
	}
} 

ISR(INT0_vect)
{
	// state = AB
	uint8_t state_current = (READ_PIN(EN_1A_PORT,EN_1A_PIN) << 1) | READ_PIN(EN_1B_PORT,EN_1B_PIN);
	if(state_current == 0 || state_current == 3)	// CW
	{
		enc1++;
	}
	else if(state_current == 1 || state_current == 2)	// CCW
	{
		enc1--;
	}
}

ISR(INT1_vect)
{
	// state = AB
	uint8_t state_current = (READ_PIN(EN_2A_PORT,EN_2A_PIN) << 1) | READ_PIN(EN_2B_PORT,EN_2B_PIN);
	if(state_current == 0 || state_current == 3)	// CW
	{
		enc2++;
	}
	else if(state_current == 1 || state_current == 2)	// CCW
	{
		enc2--;
	}
}

ISR(TIMER2_COMPA_vect)
{
	static uint8_t count = 0;
	
	if(count == 50)
	{
		// calculate speeds at 20 Hz
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			speed1 = (float)enc1 * ENC_CONST;
			speed2 = (float)enc2 * ENC_CONST;
			enc1 = enc2 = 0;
		}
		count = 0;
	}
	else
	{
		count++;
	}
}