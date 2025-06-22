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

#include "Components/ComputerComms_CDC/ComputerComms_CDC.hpp"

GPIO_Pin_STM32 ledOrange(GPIOD, GPIO_PIN_13);
GPIO_Pin_STM32 ledGreen(GPIOD, GPIO_PIN_12);
GPIO_Pin_STM32 ledRed(GPIOD, GPIO_PIN_14);
GPIO_Pin_STM32 ledBlue(GPIOD, GPIO_PIN_15);
BinLeds discoveryLeds((GPIO_Pin*[]){&ledOrange, &ledGreen, &ledRed, &ledBlue}, 4);

Queue_STM32<std::vector<uint8_t>, 4> computerPktQueue("computerPktQueue");

ComputerComms_CDC componentComputerComms(&computerPktQueue, CDC_STM32::getInstance());

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

	//Init tasks
	TaskHandle_t hTaskReceiveFromComputer;
	xTaskCreate(executableDispatch, "ReceiveFromComputer", 256, &componentComputerComms, 20, &hTaskReceiveFromComputer);

	discoveryLeds.set(3);
	vTaskDelete(nullptr);
}
