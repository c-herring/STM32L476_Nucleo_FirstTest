/*
 * PIDMotor.h
 *
 *  Created on: Mar 10, 2017
 *      Author: HeZ
 */

#ifndef PIDMOTOR_H_
#define PIDMOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

typedef struct {
	float Kp;
	float Ki;
	float Kd;
} PIDParams_TypeDef;

typedef struct {
	PIDParams_TypeDef params;

	float error;
	float prev_error;
	float Ierror;
	float Ierror_limit;
	uint32_t pidRate; // �s
	float outLim;
	uint32_t out;

} PIDControl_TypeDef;

typedef struct {
	PIDControl_TypeDef pid;

	int32_t encPos; // TODO: Add functionality to wrap-around if we go past +2^31-1 or -2^31
	int32_t lastEncPos;

	float vel; // Current velocity
	float velSet; // Velocity set point



}PIDMotor_TypeDef;


// Initialise the motor
extern void Motor_Init(PIDMotor_TypeDef *motor, PIDParams_TypeDef _pid_params, uint32_t _pidRate,  uint32_t pwmPeriod, float _Ierror_lim);

// Initialise a pid struct
extern void PID_Init(PIDControl_TypeDef *pid, PIDParams_TypeDef _pid_params);

// Computer this PID loop
extern uint32_t PID_Computer(PIDMotor_TypeDef *motor);

#ifdef __cplusplus
}
#endif

#endif /* PIDMOTOR_H_ */
