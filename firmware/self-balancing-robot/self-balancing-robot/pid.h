
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

typedef struct{
  //! Last process value, used to find derivative of process value.
  float lastProcessValue;
  //! Summation of errors, used for integrate calculations
  double sumError;
  //! The Proportional tuning constant, multiplied with SCALING_FACTOR
  float P_Factor;
  //! The Integral tuning constant, multiplied with SCALING_FACTOR
  float I_Factor;
  //! The Derivative tuning constant, multiplied with SCALING_FACTOR
  float D_Factor;
  //! Maximum allowed error, avoid overflow
  float maxError;
  //! Maximum allowed sum error, avoid overflow
  double maxSumError;
} PidData_t;

void pid_init(float p_factor, float i_factor, float d_factor, double max_i_windup, PidData_t *pid);

float pid_controller(float setPoint, float processValue, PidData_t *pid_st);

void pid_reset(PidData_t *pid_st);

#endif