/*
 * MotorDC.hpp
 *
 *  Created on: Jul 5, 2025
 *      Author: Gabriel
 */

#ifndef SHARED_ROBOFRAMEWORK_INC_MOTORDC_HPP_
#define SHARED_ROBOFRAMEWORK_INC_MOTORDC_HPP_

class MotorDC {
public:
	virtual int32_t init() = 0;
	virtual int32_t setPower(float newPower) = 0;
};

#endif /* SHARED_ROBOFRAMEWORK_INC_MOTORDC_HPP_ */
