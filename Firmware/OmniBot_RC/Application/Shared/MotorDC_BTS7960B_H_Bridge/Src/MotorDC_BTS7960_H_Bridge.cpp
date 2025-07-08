/*
 * otorDC_BTS7960B_H_Bridge.cpp
 *
 *  Created on: Jul 5, 2025
 *      Author: Gabriel
 */

#include <cmath>
#include "MotorDC_BTS7960_H_Bridge.hpp"

MotorDC_BTS7960B_H_Bridge::MotorDC_BTS7960B_H_Bridge(PWM_Pin* ina, PWM_Pin* inb, GPIO_Pin* inha, GPIO_Pin* inhb) :
			ina(ina),
			inb(inb),
			inha(inha),
			inhb(inhb)
{

}

MotorDC_BTS7960B_H_Bridge::~MotorDC_BTS7960B_H_Bridge() {

}

int32_t MotorDC_BTS7960B_H_Bridge::init() {
	int32_t errors = 0;
	errors += ina->init();
	errors += inb->init();
	errors += inha->init();
	errors += inhb->init();
	if(!errors){
		initialized = true;
	}
	return errors;
}

int32_t MotorDC_BTS7960B_H_Bridge::setPower(float newPower) {
	int32_t errors = 0;
	if(!initialized){
		// Not initialized
		return -1;
	}
	if(newPower>1 || newPower<-1){
		// Power out of range
		return -1;
	}
	if(std::isnan(newPower)){
		// Float the motor
		errors += ina->setDutyCycle(0);
		errors += inb->setDutyCycle(0);
		errors += ina->reset();
		errors += inb->reset();
		errors += inha->reset();
		errors += inhb->reset();
		return errors;
	}
	if(newPower>=0){
		errors += ina->setDutyCycle(newPower);
		errors += inb->setDutyCycle(0);
		errors += ina->reset();
		errors += inb->reset();
		errors += inha->set();
		errors += inhb->set();
	}else{
		errors += ina->setDutyCycle(0);
		errors += inb->setDutyCycle(-newPower);
		errors += ina->reset();
		errors += inb->reset();
		errors += inha->set();
		errors += inhb->set();
	}
	return errors;
}
