/*
 * OmniBotController.cpp
 *
 *  Created on: Jun 29, 2025
 *      Author: Gabriel
 */

#include "OmniBotController.hpp"

OmniBotController::OmniBotController(std::array<MotorPID*, 4> motors) :
Executable(Log::Level::Debug, "OmniBotController"),
motors(motors)
{

}

OmniBotController::~OmniBotController() {

}

int32_t OmniBotController::init() {
	return 0;
}

int32_t OmniBotController::execute() {
	while(true){

	}
	return 0;
}
