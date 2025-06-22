/*
 * ComputerComms_CDC.hpp
 *
 *  Created on: Jun 21, 2025
 *      Author: Gabriel
 */

#ifndef APPLICATION_COMPONENTS_COMPUTERCOMMSCDC_COMPUTERCOMMSCDC_HPP_
#define APPLICATION_COMPONENTS_COMPUTERCOMMSCDC_COMPUTERCOMMSCDC_HPP_

#include "Executable.hpp"
#include "Queue.hpp"
#include "UART.hpp"
#include <vector>

class ComputerComms_CDC : public Executable {
public:
	ComputerComms_CDC(Queue<std::vector<uint8_t>>* outQueue, UART* computerUart);
	virtual ~ComputerComms_CDC();
	int32_t init();
	int32_t execute();
private:
	Queue<std::vector<uint8_t>>* outQueue;
	UART* computerUart;
	static constexpr uint32_t maxPktSizeBytes = 256;
	static constexpr uint32_t queueSendTimeoutMs = 100;
	uint8_t pktBuffer[maxPktSizeBytes];
};

#endif /* APPLICATION_COMPONENTS_COMPUTERCOMMSCDC_COMPUTERCOMMSCDC_HPP_ */
