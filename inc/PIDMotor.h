/*
 * PIDMotor.h
 *
 *  Created on: Mar 6, 2017
 *      Author: HeZ
 */

#ifndef PIDMOTOR_H_
#define PIDMOTOR_H_

#include "stdint.h"

class PIDMotor {
public:

	typedef struct {
		int32_t encPos;
		int32_t lastEncPos;
		int32_t encVel;
		int32_t encVelSet;
	} _ControlState;
	_ControlState ControlState;

	typedef struct {
		float Kp;
		float Ki;
		float Kd;
		uint32_t PWM_MAX;
	}PID_Params;

	PIDMotor(PID_Params _v_PID, TIM_HandleTypeDef *_htimer, uint32_t _timChannel);
	void setVelPID(PID_Params _v_PID);
	virtual ~PIDMotor();
private:
	PID_Params v_PID;

	TIM_HandleTypeDef *htimer;
	uint32_t timChannel;
	float pError;
	float lastError;
	float iError;


};


#endif /* PIDMOTOR_H_ */
