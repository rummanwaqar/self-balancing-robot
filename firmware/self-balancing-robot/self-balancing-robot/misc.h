/*
 * misc.h
 *
 * Created: 2018-03-31 8:27:16 PM
 *  Author: rumman
 */ 


#ifndef _SBR_MISC_H_
#define _SBR_MISC_H_

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

#endif /* MISC_H_ */