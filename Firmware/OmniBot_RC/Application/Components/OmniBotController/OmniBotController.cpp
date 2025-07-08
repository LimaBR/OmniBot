/*
 * OmniBotController.cpp
 *
 *  Created on: Jun 29, 2025
 *      Author: Gabriel
 */

#include "OmniBotController.hpp"
#include <cmath>
#include <numbers>

OmniBotController::OmniBotController(std::array<MotorDC*, 4> motors,
		std::array<Encoder*, 4> encoders,
		SoftTimer* pidTimer,
		Queue<RobotCommandPacket>* robotPacketQueue) :
		Executable(Log::Level::Debug, "OmniBotController"),
		motors(motors),
		encoders(encoders),
		pidTimer(pidTimer),
		robotPacketQueue(robotPacketQueue)
{

}

OmniBotController::~OmniBotController() {

}

int32_t OmniBotController::init() {
	int32_t errors = 0;
	log(Log::Level::Informational, "Initializing");
	for(uint32_t i=0; i<numMotors; i++){
		errors += motors.at(i)->init();
		errors += encoders.at(i)->init();
	}
	errors += pidTimer->init();
	log(Log::Level::Informational, "Initialized with status %d", errors);
	return errors;
}

int32_t OmniBotController::execute() {
	log(Log::Level::Informational, "Starting execution");
	if(pidTimer->start(10, true) < 0){
		log(Log::Level::Error, "pidTimer failed to start");
		return -1;
	}
	motors.at(0)->setPower(0);
	motors.at(1)->setPower(0);
	motors.at(2)->setPower(0);
	motors.at(3)->setPower(0);
	while(true){
		RobotCommandPacket command;
		if(robotPacketQueue->receive(&command, 0) >= 0){
			log(Log::Level::Debug, "PKT RECV: Vt = %f, Vn = %f, Va = %f", command.tangentSpeed, command.normalSpeed, command.angularSpeed);
			state = command;
		}
		processPID();
		pidTimer->waitForCompletion(0xFFFFFFFF);
	}
	return 0;
}

// Private methods

int32_t OmniBotController::processPID() {
	int32_t errors = 0;
	static int32_t debugLog = 0;
	constexpr float powerLimit = 0.5; // FIXME To preserve the hardware
	for(uint32_t i=0; i<numMotors; i++){
		float targetWheelSpeed = state.normalSpeed * robotGeometry.wheelSines.at(i) +
				state.tangentSpeed * robotGeometry.wheelCosines.at(i) +
				state.angularSpeed * robotGeometry.robotRadius;
		float deltaTime = 0.01f; // FIXME Definir como constante da classe

		static std::array<int32_t, numMotors> currentEncoderCountsArray = {0, 0, 0, 0};
		int32_t lastEncoderCounts = currentEncoderCountsArray.at(i);
		errors += encoders.at(i)->getPosition(&currentEncoderCountsArray.at(i));
		currentEncoderCountsArray.at(i) &= 0x0000FFFF;	// Take only least significant 16 bits
		int16_t deltaEncoderCounts = static_cast<int16_t>(currentEncoderCountsArray.at(i) - lastEncoderCounts);
		float deltaRadians = static_cast<float>(deltaEncoderCounts) * 2.0f * std::numbers::pi_v<float> / robotGeometry.wheelGeometry.encoderCPR;
		float currentMotorAngularSpeed = deltaRadians / deltaTime;
		float currentWheelAngularSpeed = currentMotorAngularSpeed * robotGeometry.wheelGeometry.reductionRatio;
		float currentWheelSpeed = currentWheelAngularSpeed * robotGeometry.wheelGeometry.wheelRadius;

		constexpr float kp = 10;
		constexpr float ki = 100;
		constexpr float kd = 0;

		static std::array<float, numMotors> perror = {0, 0, 0, 0};
		float lastError = perror.at(i);
		perror.at(i) = targetWheelSpeed - currentWheelSpeed;
		static std::array<float, numMotors> ierror = {0, 0, 0, 0};
		ierror.at(i) += perror.at(i) * deltaTime;
		// Clamp ierror.at (anti-windup)
		constexpr float ilimit = powerLimit/ki;
		if(ierror.at(i) > ilimit){
			ierror.at(i) = ilimit;
		}else if(ierror.at(i) < -ilimit){
			ierror.at(i) = -ilimit;
		}
		float derror = (perror.at(i) - lastError) / deltaTime;

		float motorPower = kp*perror.at(i) + kd*derror + ki*ierror.at(i);
		// Clamp motorPower
		if(motorPower > powerLimit){
			motorPower = powerLimit;
		}else if(motorPower < -powerLimit){
			motorPower = -powerLimit;
		}
		errors += motors.at(i)->setPower(motorPower);
		if(debugLog == 10){
			log(Log::Level::Debug, "M%u kp*perror = %f, ki*ierror = %f, kd*derror = %f", i, kp*perror.at(i), ki*ierror.at(i), kd*derror);
			debugLog = 0;
		}else{
			debugLog++;
		}
	}
	return errors;
}
