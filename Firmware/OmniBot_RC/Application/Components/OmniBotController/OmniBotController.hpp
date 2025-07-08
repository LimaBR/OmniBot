/*
 * OmniBotController.hpp
 *
 *  Created on: Jun 29, 2025
 *      Author: Gabriel
 */

#ifndef COMPONENTS_OMNIBOTCONTROLLER_OMNIBOTCONTROLLER_HPP_
#define COMPONENTS_OMNIBOTCONTROLLER_OMNIBOTCONTROLLER_HPP_

#include <Executable.hpp>
#include <MotorDC.hpp>
#include <Encoder.hpp>
#include <SoftTimer.hpp>
#include <Queue.hpp>
#include "../../RobotCommandPacket.hpp"

#include <array>

class OmniBotController : public Executable {
private:
	static constexpr uint32_t numMotors = 4;
	class RobotGeometry{
	public:
		class WheelGeometry{
		public:
			// TODO Conferir, pois parece estar 1s mais rápido
			float wheelRadius = 0.029f;
			float reductionRatio = 1.0f/90.0f;
			int32_t encoderCPR = -52;
		}wheelGeometry;
		float robotRadius = 0.1045f;
		static constexpr std::array<float, numMotors> wheelSines = {
			0.7071067812f,  // sin(45°)
			0.7071067812f,  // sin(135°)
			-0.7071067812f, // sin(225°)
			-0.7071067812f  // sin(315°)
		};
		static constexpr std::array<float, numMotors> wheelCosines = {
			0.7071067812f,  // cos(45°)
			-0.7071067812f, // cos(135°)
			-0.7071067812f, // cos(225°)
			0.7071067812f   // cos(315°)
		};
	}robotGeometry;
public:
	OmniBotController(std::array<MotorDC*, numMotors> motors,
			std::array<Encoder*, numMotors> encoders,
			SoftTimer* pidTimer,
			Queue<RobotCommandPacket>* robotPacketQueue);
	virtual ~OmniBotController();
	int32_t init();
	int32_t execute();
private:
	int32_t processPID();
	std::array<MotorDC*, numMotors> motors;
	std::array<Encoder*, numMotors> encoders;
	SoftTimer* pidTimer;
	Queue<RobotCommandPacket>* robotPacketQueue;
	RobotCommandPacket state;
};

#endif /* COMPONENTS_OMNIBOTCONTROLLER_OMNIBOTCONTROLLER_HPP_ */
