/*
 * PIDMotor.c
 *
 *  Created on: Mar 10, 2017
 *      Author: HeZ
 */

#include "PIDMotor.h"





// Initialise the motor and PID structure. This must be called before running the PID
void Motor_Init(PIDMotor_TypeDef *motor, PIDParams_TypeDef _pid_params, uint32_t _pidRate)
{
	// Copy the PID parameters into the structure
	//PID_Init(&motor->pid, _pid_params, _pidRate);

	// Set intial values
	motor->encPos = 0;
	motor->lastEncPos = 0;
	motor->vel = 0.0f;
	motor->velSet = 0.0f;
}

// Initialise the PID strucutre
void PID_Init(PIDControl_TypeDef *pid, PIDParams_TypeDef _pid_params, uint32_t _pidRate)
{
	pid->params.Kp = _pid_params.Kp;
	pid->params.Ki = _pid_params.Ki;
	pid->params.Kd = _pid_params.Kd;
	pid->pidRate = _pidRate;
}

