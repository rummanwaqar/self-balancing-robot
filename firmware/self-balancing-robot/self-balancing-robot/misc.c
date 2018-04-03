
/*
 * misc.c
 *
 * Created: 2018-04-01 5:12:15 PM
 *  Author: rumma
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
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

Vector3 toEulerAngle(const float q0, const float q1, const float q2, const float q3)
{
	Vector3 rpy;
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