
/*
 * pid.h
 *  Author: rumman
 * Inspiration from ATMEL AVR221 - Discrete PID controller
 */ 

/*
 * Setpoints and data used by the PID control algorithm
 */

#include "defines.h"

#ifndef _SBR_PID_H_
#define _SBR_PID_H_

void pid_init(float p_factor, float i_factor, float d_factor, double max_i_windup, float effort_limit, PidData_t *pid);

float pid_controller(float setPoint, float processValue, PidData_t *pid_st);

void pid_reset(PidData_t *pid_st);

#endif