
/*
 * pid.c
 *
 * Created: 2018-04-02 11:37:58 PM
 *  Author: rumman
 */ 

#include "pid.h"

void pid_init(float p_factor, float i_factor, float d_factor, double max_i_windup, float effort_limit, PidData_t *pid)
{
	// Start values for PID controller
	pid->sumError = 0.0;
	pid->lastProcessValue = 0.0f;
	// Tuning constants for PID loop
	pid->P_Factor = p_factor;
	pid->I_Factor = i_factor;
	pid->D_Factor = d_factor;
	// Limits to avoid overflow
	pid->maxSumError = max_i_windup;
	pid->maxEffort = effort_limit;
}

float pid_controller(float setPoint, float processValue, PidData_t *pid_st)
{
	float error, p_term, d_term, i_term, effort;

	error = setPoint - processValue;

	// Calculate p term
	p_term = pid_st->P_Factor * error;

	// Calculate I term and limit integral runaway
	pid_st->sumError += error;
	if(pid_st->sumError > pid_st->maxSumError) pid_st->sumError = pid_st->maxSumError;
	if(pid_st->sumError < (-1 * pid_st->maxSumError)) pid_st->sumError = -1 * pid_st->maxSumError;
	i_term = pid_st->I_Factor * pid_st->sumError;

	// Calculate D term
	d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);

	pid_st->lastProcessValue = processValue;

	effort = p_term + i_term + d_term;
	if(effort > pid_st->maxEffort) effort = MOTOR_MAX_EFFORT;
	if(effort < -1 * MOTOR_MAX_EFFORT) effort = -1 * MOTOR_MAX_EFFORT;
	
	return effort;
}

void pid_reset(PidData_t *pid_st)
{
	pid_st->sumError = 0.0;
	pid_st->lastProcessValue = 0.0f;
}