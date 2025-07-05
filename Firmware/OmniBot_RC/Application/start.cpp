/*
 * start.cpp
 *
 *  Created on: Oct 31, 2024
 *      Author: Gabriel
 */

#include "start.h"
#include <cstdio>
#include "FreeRTOS.h"
#include "GPIO_Pin_STM32.hpp"
#include "BinLeds.hpp"
#include "LogDriver.hpp"
#include "GPIO_Pin_STM32.hpp"
#include "PWM_Pin_STM32.hpp"
#include "UART_STM32.hpp"
#include "Queue_STM32.hpp"
#include "CDC_STM32.hpp"
#include "MotorPID_STM32.hpp"
#include "Encoder_STM32.hpp"

#include "Components/ComputerComms_CDC/ComputerComms_CDC.hpp"
#include "Components/OmniBotController/OmniBotController.hpp"

// Leds
GPIO_Pin_STM32 ledOrange(GPIOD, GPIO_PIN_13);
GPIO_Pin_STM32 ledGreen(GPIOD, GPIO_PIN_12);
GPIO_Pin_STM32 ledRed(GPIOD, GPIO_PIN_14);
GPIO_Pin_STM32 ledBlue(GPIOD, GPIO_PIN_15);
BinLeds discoveryLeds((GPIO_Pin*[]){&ledOrange, &ledGreen, &ledRed, &ledBlue}, 4);

// Encoders
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
Encoder_STM32 encoder0(&htim2);
Encoder_STM32 encoder1(&htim3);
Encoder_STM32 encoder2(&htim4);
Encoder_STM32 encoder3(&htim5);
std::array<Encoder*, 4> encoders = {&encoder0, &encoder1, &encoder2, &encoder3};

// Motors
MotorPID_STM32::Configuration motorConfig = {
		.wheelRadius = 40,
		.reductionRatio = 1/72,
		.encoderCountsPerRevolution = 52,
};
MotorPID_STM32 motor0(motorConfig);
MotorPID_STM32 motor1(motorConfig);
MotorPID_STM32 motor2(motorConfig);
MotorPID_STM32 motor3(motorConfig);
std::array<MotorPID*, 4> motors = {&motor0, &motor1, &motor2, &motor3};

// Queues
Queue_STM32<std::vector<uint8_t>, 4> computerPktQueue("computerPktQueue");

// Components
ComputerComms_CDC componentComputerComms(&computerPktQueue, CDC_STM32::getInstance());
OmniBotController componentOmniBotController(motors);

void executableDispatch(void* _executable){
	Executable* executable = static_cast<Executable*>(_executable);
	executable->execute();
	vTaskDelete(NULL);
}

void start(){
	discoveryLeds.set(1);
	printf("----------------INIT----------------\n");
	printf("X and Y: %lu\n", HAL_GetUIDw0());
	printf("Wafer number: %c\n", (uint8_t)HAL_GetUIDw1());
	printf("Lot number: %c%c%c%c%c%c%c\n",
			(uint8_t)(HAL_GetUIDw1()>>8),
			(uint8_t)(HAL_GetUIDw1()>>16),
			(uint8_t)(HAL_GetUIDw1()>>24),
			(uint8_t)(HAL_GetUIDw2()),
			(uint8_t)(HAL_GetUIDw2()>>8),
			(uint8_t)(HAL_GetUIDw2()>>16),
			(uint8_t)(HAL_GetUIDw2()>>24)
			);
	discoveryLeds.set(2);

	// Init log facility
	LogDriver::getInstance()->init();
	TaskHandle_t hTaskLogDriver;
	xTaskCreate(executableDispatch, "LogDriver", 256, LogDriver::getInstance(), 5, &hTaskLogDriver);

	// Init queues
	computerPktQueue.init();

	// Init shared resources
	CDC_STM32::getInstance()->init();

	//Init components
	componentComputerComms.init();
	componentOmniBotController.init();

	//Init tasks
	TaskHandle_t hTaskReceiveFromComputer;
	xTaskCreate(executableDispatch, "ReceiveFromComputer", 256, &componentComputerComms, 20, &hTaskReceiveFromComputer);

	TaskHandle_t hTaskOmniBotController;
	xTaskCreate(executableDispatch, "OmniBotController", 256, &componentOmniBotController, 20, &hTaskOmniBotController);

	discoveryLeds.set(3);
	vTaskDelete(nullptr);
}
