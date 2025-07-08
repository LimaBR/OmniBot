/*
 * MotorDC_BTS7960B_H_Bridge.hpp
 *
 *  Created on: Jul 5, 2025
 *      Author: Gabriel
 */

#ifndef MOTORDC_BTS7960B_H_BRIDGE_HPP_
#define MOTORDC_BTS7960B_H_BRIDGE_HPP_

#include "main.h"
#include "MotorDC.hpp"
#include "PWM_Pin.hpp"
#include "GPIO_Pin.hpp"

class MotorDC_BTS7960B_H_Bridge : public MotorDC {
public:
	MotorDC_BTS7960B_H_Bridge(PWM_Pin* ina, PWM_Pin* inb, GPIO_Pin* inha, GPIO_Pin* inhb);
	virtual ~MotorDC_BTS7960B_H_Bridge();
	int32_t init();
	int32_t setPower(float newPower);
private:
	PWM_Pin* ina;
	PWM_Pin* inb;
	GPIO_Pin* inha;
	GPIO_Pin* inhb;
	bool initialized = false;
};

#endif /* MOTORDC_BTS7960B_H_BRIDGE_HPP_ */
