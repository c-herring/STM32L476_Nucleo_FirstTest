/*
 * PIDMotor.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: HeZ
 */

#include "stm32l4xx_hal.h"
#include "PIDMotor.h"




PIDMotor::PIDMotor(PID_Params _v_PID,  TIM_HandleTypeDef *_htimer, uint32_t _timChannel) {
	// Initialise the motor params
	ControlState.encPos = 0;
	ControlState.encVel = 0;
	ControlState.encVelSet = 0;

	setVelPID(_v_PID);

	htimer = _htimer;
	timChannel = _timChannel;


}

void PIDMotor::setVelPID(PID_Params _v_PID)
{
	v_PID.Kp = _v_PID.Kp;
	v_PID.Ki = _v_PID.Ki;
	v_PID.Kd = _v_PID.Kd;
	v_PID.PWM_MAX = _v_PID.PWM_MAX;
}

PIDMotor::~PIDMotor() {
	// TODO Auto-generated destructor stub
}

