/*
 * ARMs_Protocol.h
 *
 *  Created on: Jan 21, 2022
 *      Author: Natmatee
 */

#ifndef INC_ARMSPROTOCOL_H_
#define INC_ARMSPROTOCOL_H_

/* USER CODE BEGIN Includes */
#include "usart.h"
//#include "crc.h"
#include "gpio.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define ARMsProtocol_HEADER							0xFF

//Recieve command
#define ARMsProtocol_ADDR_JOINTJOG 					0x01
#define ARMsProtocol_ADDR_CATESIANJOG 				0x02
#define ARMsProtocol_ADDR_TRAJECTORY 				0x03
#define ARMsProtocol_ADDR_PATHWAY					0x04
#define ARMsProtocol_ADDR_GRIPPER			 		0x05
#define ARMsProtocol_ADDR_JOTNTFEEDBACK				0x06
#define ARMsProtocol_ADDR_BOARDFEEDBACK				0x07

//Transmit command
#define ARMsProtocol_TRANSMIT_ILLEGALFUNC			0x01
#define ARMsProtocol_TRANSMIT_ILLEGALCRC			0x02
#define ARMsProtocol_TRANSMIT_ACKNOWLEDGE			0x03
#define ARMsProtocol_TRANSMIT_DONE					0x04
#define ARMsProtocol_TRANSMIT_FEEDBACK				0x05

/* USER CODE END Private defines */

typedef struct {
	UART_HandleTypeDef 	*handle;
	USART_TypeDef 		*Instance;
	uint8_t 			slave_id;

} ARMsProtocol_HandleTypedef;

typedef struct {
	uint8_t Rx_buf[100];
	uint8_t Rx_reg;
	uint8_t Tx_buf[15];
	uint8_t _CRC;
	uint32_t CRC_CAL;
	uint8_t Header;
	uint8_t Id;
	uint8_t Length;
	uint8_t Instruction;
	uint8_t Data_buf[100];
	uint8_t State;
	uint8_t Flag;
	uint8_t Tx_flag;
	uint8_t Code;
	uint8_t Rx_count;
	uint8_t Tx_count;
	uint8_t Jointjog_flag;
	uint8_t Catesian_flag;
	uint8_t Trajectory_flag;
	uint8_t Pathway_flag;

} ARMsProtocol_DATA;

// Main FUNC
extern void ARMsProtocol_FUNC_Init(void);
extern void ARMsProtocol_FUNC_Interface(void);
extern void ARMsProtocol_FUNC_Rx_Callback(UART_HandleTypeDef *huart);
extern void ARMsProtocol_FUNC_Tx_Callback(UART_HandleTypeDef *huart);
extern void ARMsProtocol_FUNC_Rx_Clrbuf(uint8_t count);
extern void ARMsProtocol_FUNC_Data_Clrbuf(void);
// Command FUNC
extern void ARMsProtocol_FUNC_Jointjog(void);
extern void ARMsProtocol_FUNC_Catesianjog(void);
extern void ARMsProtocol_FUNC_Trajectory(void);
extern void ARMsProtocol_FUNC_Gripper(void);
extern void ARMsProtocol_FUNC_Pathway(void);
extern void ARMsProtocol_FUNC_Jointfeedback(UART_HandleTypeDef *huart);
extern void ARMsProtocol_FUNC_Boardfeedback(UART_HandleTypeDef *huart);
extern void ARMsProtocol_FUNC_Clrflag(void);

extern void ARMsProtocol_CALC_CRC(uint32_t *pBuffer, uint32_t BufferLength);

extern void ARMsProtocol_EXCEPTION_Response(UART_HandleTypeDef *huart, uint8_t code);
#endif /* INC_ARMSPROTOCOL_H_ */
