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
#include "Encoder_STM32.hpp"
#include "SoftTimer_STM32.hpp"
#include "Components/ComputerComms_CDC/ComputerComms_CDC.hpp"
#include "Components/OmniBotController/OmniBotController.hpp"
#include "MotorDC_BTS7960_H_Bridge.hpp"
#include "RobotCommandPacket.hpp"

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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
PWM_Pin_STM32 m0_ina(&htim8, TIM_CHANNEL_3);
PWM_Pin_STM32 m0_inb(&htim8, TIM_CHANNEL_4);
PWM_Pin_STM32 m1_ina(&htim1, TIM_CHANNEL_1);
PWM_Pin_STM32 m1_inb(&htim1, TIM_CHANNEL_2);
PWM_Pin_STM32 m2_ina(&htim1, TIM_CHANNEL_3);
PWM_Pin_STM32 m2_inb(&htim1, TIM_CHANNEL_4);
PWM_Pin_STM32 m3_ina(&htim8, TIM_CHANNEL_1);
PWM_Pin_STM32 m3_inb(&htim8, TIM_CHANNEL_2);
GPIO_Pin_STM32 m0_inha(M0_LEN_GPIO_Port, M0_LEN_Pin);
GPIO_Pin_STM32 m0_inhb(M0_REN_GPIO_Port, M0_REN_Pin);
GPIO_Pin_STM32 m1_inha(M1_LEN_GPIO_Port, M1_LEN_Pin);
GPIO_Pin_STM32 m1_inhb(M1_REN_GPIO_Port, M1_REN_Pin);
GPIO_Pin_STM32 m2_inha(M2_LEN_GPIO_Port, M2_LEN_Pin);
GPIO_Pin_STM32 m2_inhb(M2_REN_GPIO_Port, M2_REN_Pin);
GPIO_Pin_STM32 m3_inha(M3_LEN_GPIO_Port, M3_LEN_Pin);
GPIO_Pin_STM32 m3_inhb(M3_REN_GPIO_Port, M3_REN_Pin);
MotorDC_BTS7960B_H_Bridge dcmotor0(&m0_ina, &m0_inb, &m0_inha, &m0_inhb);
MotorDC_BTS7960B_H_Bridge dcmotor1(&m1_ina, &m1_inb, &m1_inha, &m1_inhb);
MotorDC_BTS7960B_H_Bridge dcmotor2(&m2_ina, &m2_inb, &m2_inha, &m2_inhb);
MotorDC_BTS7960B_H_Bridge dcmotor3(&m3_ina, &m3_inb, &m3_inha, &m3_inhb);
std::array<MotorDC*, 4> dcmotors = {&dcmotor0, &dcmotor1, &dcmotor2, &dcmotor3};

// Queues
Queue_STM32<std::vector<uint8_t>, 4> computerPktQueue("computerPktQueue");
Queue_STM32<RobotCommandPacket, 1> robotPacketQueue("robotPacketQueue");

// Timers
SoftTimer_STM32 pidTimer(nullptr, "pidTimer");

// Components
ComputerComms_CDC componentComputerComms(&computerPktQueue, CDC_STM32::getInstance());
OmniBotController componentOmniBotController(dcmotors, encoders, &pidTimer, &robotPacketQueue);

void executableDispatch(void* _executable){
	Executable* executable = static_cast<Executable*>(_executable);
	executable->execute();
	vTaskDelete(NULL);
}

void start(){
	discoveryLeds.set(1);
	printf("----------------INIT----------------\n");
	printf("X and Y: %u, %u\n", (uint16_t)HAL_GetUIDw0(), (uint16_t)HAL_GetUIDw0()>>16);
	printf("Wafer number: %u\n", (uint8_t)HAL_GetUIDw1());
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
	robotPacketQueue.init();
	RobotCommandPacket testPkt = {
			.tangentSpeed = 0,
			.normalSpeed = 0,
			.angularSpeed = 1
	};
	robotPacketQueue.send(testPkt, 0);

	// Init shared resources
	CDC_STM32::getInstance()->init();

	//Init components
	componentComputerComms.init();
	componentOmniBotController.init();

	//Init tasks
	TaskHandle_t hTaskReceiveFromComputer;
	xTaskCreate(executableDispatch, "ReceiveFromComputer", 256, &componentComputerComms, 20, &hTaskReceiveFromComputer);

	TaskHandle_t hTaskOmniBotController;
	xTaskCreate(executableDispatch, "OmniBotController", 256, &componentOmniBotController, 21, &hTaskOmniBotController);

	discoveryLeds.set(3);
	vTaskDelete(nullptr);
}
