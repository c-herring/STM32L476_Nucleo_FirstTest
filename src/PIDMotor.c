/*
 * PIDMotor.c
 *
 *  Created on: Mar 10, 2017
 *      Author: HeZ
 */

#include "PIDMotor.h"





// Initialise the motor and PID structure. This must be called before running the PID
void Motor_Init(PIDMotor_TypeDef *motor, PIDParams_TypeDef _pid_params, uint32_t _pidRate, uint32_t _pwmPeriod, float _Ierror_lim)
{
	// Copy the PID parameters into the structure
	PID_Init(&motor->pid, _pid_params);
	motor->pid.pidRate = _pidRate;
	motor->pid.outLim = _pwmPeriod;

	motor->pid.error = 0.0f;
	motor->pid.prev_error = 0.0f;
	motor->pid.Ierror = 0.0f;
	motor->pid.Ierror_limit = _Ierror_lim;

	// Set intial values
	motor->encPos = 0;
	motor->lastEncPos = 0;
	motor->vel = 0.0f;
	motor->velSet = 0.0f;
}

// Initialise the PID strucutre
void PID_Init(PIDControl_TypeDef *pid, PIDParams_TypeDef _pid_params)
{
	pid->params.Kp = _pid_params.Kp;
	pid->params.Ki = _pid_params.Ki;
	pid->params.Kd = _pid_params.Kd;

}

// Compute PID
extern uint32_t PID_Computer(PIDMotor_TypeDef *motor)
{
	motor->vel = (float) (motor->encPos - motor->lastEncPos)
}
