/*
 * OmniBotController.hpp
 *
 *  Created on: Jun 29, 2025
 *      Author: Gabriel
 */

#ifndef COMPONENTS_OMNIBOTCONTROLLER_OMNIBOTCONTROLLER_HPP_
#define COMPONENTS_OMNIBOTCONTROLLER_OMNIBOTCONTROLLER_HPP_

#include <Executable.hpp>
#include <MotorPID.hpp>

#include <array>

class OmniBotController : public Executable {
public:
	OmniBotController(std::array<MotorPID*, 4> motors);
	virtual ~OmniBotController();
	int32_t init();
	int32_t execute();
private:
	std::array<MotorPID*, 4> motors;
};

#endif /* COMPONENTS_OMNIBOTCONTROLLER_OMNIBOTCONTROLLER_HPP_ */
