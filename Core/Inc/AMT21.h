/*
 * AMT21.h
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#ifndef INC_AMT21_H_
#define INC_AMT21_H_

/*
 * Include
 */
#include "stm32h7xx_hal.h"
#include "stdint.h"

/*
 * Define
 */
#define AMT21_SET_ZERO 0x75


/*
 * STRUCT
 */
typedef struct {
	UART_HandleTypeDef *uartHandle;
	GPIO_TypeDef *DE_port;
	uint16_t DE_pin;
	uint8_t address;

	uint16_t uart_buf;
	uint16_t position;
	uint16_t prev_position;
	uint8_t k0;
	uint8_t k1;
} AMT21;


/*
 * FUNCTIONS
 */
void AMT21_initialise(AMT21 *dev, UART_HandleTypeDef *uartHandle,
		uint8_t address, GPIO_TypeDef *DE_port, uint16_t DE_Pin);
void AMT21_read_value(AMT21 *dev);
void AMT21_set_zero(AMT21 *dev);
HAL_StatusTypeDef AMT21_check_value(AMT21 *dev);
int32_t AMT21_unwrap(int32_t pulse, int32_t prev_pulse);

#endif /* INC_AMT21_H_ */
