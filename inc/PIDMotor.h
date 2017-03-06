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
	PIDMotor();
	virtual ~PIDMotor();
	typedef struct {
		uint32_t encPos;
		uint32_t encVel;
		uint32_t encVelSet;
	} ControlState;

private:


};


#endif /* PIDMOTOR_H_ */
