/*
 * motor.c
 *
 * Created: 2018-03-31 7:12:56 PM
 *  Author: rumma
 */ 

#include "defines.h"
#include <avr/io.h>
#include <util/atomic.h>

#include "motor.h"

// private functions 
void init_encoders(void);
void motor_timer_init(void);

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

void motor_timer_init(void)
{
	// enable timer2 for speed calculation
	TCCR2A |= (1<<WGM21)|(1<<COM2A1);	// CTC mode
	TCCR2B |= (1<<CS22);				// ps=64;
	TIMSK2 |= (1<< OCIE2A);
	OCR2A = 229;						// 1 kHz
}

void motor_speed(int16_t* motor1, int16_t* motor2)
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