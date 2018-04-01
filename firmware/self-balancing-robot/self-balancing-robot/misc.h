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

#endif /* MISC_H_ */