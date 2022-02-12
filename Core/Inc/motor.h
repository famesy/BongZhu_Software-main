/*
 * stepper_motor.h
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

/*
 * Include
 */
#include "stm32h7xx_hal.h"
#include "math.h"

/*
 * Define
 */

#define MIN_FREQUENCY 8.0
#define MAX_FREQUENCY 500000.0

/*
 * STRUCT
 */
typedef struct {
	/* TIM handle */
	TIM_HandleTypeDef *timHandle;
	uint32_t tim_channel;
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;
	/* memory */
	float freq;
	uint8_t duty_cycle;
	uint8_t dir_mode;
} Stepper_Motor;

typedef struct {
	/* TIM handle */
	TIM_HandleTypeDef *timHandle;
	uint32_t tim_channel;
	/* memory */
	uint8_t degree;
} Servo_Motor;

/*
 * INITIALISATION
 */
void stepper_initialise(Stepper_Motor *dev, TIM_HandleTypeDef *timHandle,
		uint32_t tim_channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, uint8_t dir_mode);
void servo_initialise(Servo_Motor *dev, TIM_HandleTypeDef *timHandle,
		uint32_t tim_channel);

/*
 * Driver FUNCTIONS
 */

void servo_set_degree(Servo_Motor *dev, uint8_t degree);
void stepper_set_speed(Stepper_Motor *dev, float freq);
/*
 * LOW-LEVEL FUNCTIONS
 */
void set_pwm(TIM_HandleTypeDef *tim_pwm, uint32_t tim_channel, float freq, float duty_cycle);

#endif /* INC_MOTOR_H_ */
