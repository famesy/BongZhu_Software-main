/*
 * AMT21.c
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */
#include "AMT21.h"

void AMT21_initialise(AMT21 *dev, UART_HandleTypeDef *uartHandle,
		uint8_t address, GPIO_TypeDef *DE_port, uint16_t DE_Pin) {
	dev->uartHandle = uartHandle;
	dev->DE_port = DE_port;
	dev->DE_pin = DE_Pin;
	dev->address = address;

	dev->uart_buf = 0;
	dev->position = 0;
	dev->prev_position = 0;
	dev->k0 = 0;
	dev->k1 = 0;
}

void AMT21_read_value(AMT21 *dev) {
	/*
	 AMT21_read_value does read raw data from encoder but you must use AMT21_check_value first.

	 :param dev = AMT21 struct
	 :return: None
	 */
	HAL_GPIO_WritePin(dev->DE_port, dev->DE_pin, 1);
	//HAL_UART_Transmit(dev->uartHandle, (uint8_t*) &(dev->address),
			//1, 100);
	HAL_UART_Transmit(dev->uartHandle, &(dev->address),
				1, 100);
	HAL_GPIO_WritePin(dev->DE_port, dev->DE_pin, 0);
	HAL_UART_Receive(dev->uartHandle, (uint8_t*) &(dev->uart_buf), 2, 100);
	dev->k0 = (dev->uart_buf & 0x4000) == 0x4000;
	dev->k1 = (dev->uart_buf & 0x8000) == 0x8000;
}

void AMT21_set_zero(AMT21 *dev) {
	/*
	 AMT21_set_zero does set encoder to zero position.

	 :param dev = AMT21 struct
	 :return: None
	 */
	uint8_t set_zero_command[2] = {(dev->address + 0x02), AMT21_SET_ZERO};
 	HAL_GPIO_WritePin(dev->DE_port, dev->DE_pin, 1);
	HAL_UART_Transmit(dev->uartHandle, (uint8_t*) set_zero_command,
			sizeof(set_zero_command), 100);
	HAL_GPIO_WritePin(dev->DE_port, dev->DE_pin, 0);
}

void AMT21_reset(AMT21 *dev) {
	/*
	 AMT21_set_zero does reset encoder.
	 :param dev = AMT21 struct
	 :return: None
	 */
	uint8_t set_zero_command[2] = {(dev->address + 0x02), 0x75};
 	HAL_GPIO_WritePin(dev->DE_port, dev->DE_pin, 1);
	HAL_UART_Transmit(dev->uartHandle, (uint8_t*) set_zero_command,
			sizeof(set_zero_command), 100);
	HAL_GPIO_WritePin(dev->DE_port, dev->DE_pin, 0);
}

HAL_StatusTypeDef AMT21_check_value(AMT21 *dev) {
	/*
	 AMT21_read_value does check correctness of your data then save to dev->position.

	 :param dev = AMT21 struct
	 :return: HAL_OK 	: if value is right
	 HAL_ERROR : if value is wrong
	 */
	uint16_t position_temp = dev->uart_buf & 0x3FFF;
	uint8_t k0_check = dev->uart_buf & 0x0001;
	uint8_t k1_check = (dev->uart_buf >> 1) & 0x0001;
	for (uint8_t i = 0; i < 6; i++) {
		dev->uart_buf = dev->uart_buf >> 2;
		k0_check ^= dev->uart_buf & 0x0001;
		k1_check ^= (dev->uart_buf >> 1) & 0x0001;
	}
	k0_check = !k0_check;
	k1_check = !k1_check;
	if ((dev->k0 == k0_check) && (dev->k1 == k1_check)) {
		dev->position = position_temp;
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

int32_t AMT21_unwrap(int32_t pulse, int32_t prev_pulse) {
	int32_t dPulse = 0;
	if (pulse - prev_pulse > 8191) {
		dPulse = -(16383 - (pulse-prev_pulse));
	} else if ( pulse -  prev_pulse < -8191) {
		dPulse = 16383 - (prev_pulse - pulse);
	} else {
		dPulse =  pulse -  prev_pulse;
	}
	return dPulse;
}
