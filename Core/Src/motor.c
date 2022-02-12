/*
 * stepper_motor.c
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#include "motor.h"

void stepper_initialise(Stepper_Motor *dev, TIM_HandleTypeDef *timHandle,
		uint32_t tim_channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, uint8_t dir_mode) {

	/* Set struct parameters */
	dev->timHandle = timHandle;
	dev->tim_channel = tim_channel;
	dev->dir_port = dir_port;
	dev->dir_pin = dir_pin;
	/*
	 * dir mode set direction of stepper
	 */
	dev->dir_mode = dir_mode;
	HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, 0);
	HAL_TIM_PWM_Start(dev->timHandle, dev->tim_channel);
	dev->freq = 1;
	stepper_set_speed(dev, 0);
}

void servo_initialise(Servo_Motor *dev, TIM_HandleTypeDef *timHandle,uint32_t tim_channel) {
	/* Set struct parameters */
	dev->timHandle = timHandle;
	dev->tim_channel = tim_channel;
	HAL_TIM_PWM_Start(dev->timHandle, dev->tim_channel);
	dev->degree = 1;
	servo_set_degree(dev, 0);
}

void set_pwm(TIM_HandleTypeDef *tim_pwm, uint32_t tim_channel, float freq,
		float duty_cycle) {
	/*
	 set_pwm does set pwm timer to your specific value.

	 :param freq = frequency of pwm
	 :param duty_cycle is % duty cycle 0.0 - 1.0
	 :return: None
	 */
	if (freq < MIN_FREQUENCY){
		freq = MIN_FREQUENCY;
	}
	else if (freq > MAX_FREQUENCY){
		freq = MAX_FREQUENCY;
	}
	uint16_t ARR_value = 500000 / freq; //500000 come from 275MHz/550
	uint16_t CCRx_value = (ARR_value * duty_cycle);
	if (duty_cycle == 1.0) {
		CCRx_value = 0;
	}
	__HAL_TIM_SET_AUTORELOAD(tim_pwm, ARR_value);
	__HAL_TIM_SET_COMPARE(tim_pwm, tim_channel, CCRx_value);
}

void servo_set_degree(Servo_Motor *dev, uint8_t degree) {
	/*
	 servo_set_degree does set your servo to your given value.

	 :param degree is degree of servo motor (0-180)
	 :return: None
	 */
	if (degree != dev->degree) {
		if (degree > 180) {
			degree = 180.0;
		} else if (degree < 0) {
			degree = 0.0;
		}
		float cyc = 0.05 + ((degree / 180.0) * 0.05);
		set_pwm(dev->timHandle, dev->tim_channel, 50, cyc);
		dev->degree = degree;
	}
}

void stepper_set_speed(Stepper_Motor *dev, float freq) {
	/*
	 stepper_set_speed does set your stepper to your given value.

	 :param freq can be -9999.9999 to 9999.9999. signed value use to set stepper direction.
	 :return: None
	 */
	if (freq != dev->freq) {
		if (freq > MIN_FREQUENCY) {
			if (dev->dir_mode == 0){
				HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, 0);
			}
			else if (dev->dir_mode == 1){
				HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, 1);
			}
			set_pwm(dev->timHandle, dev->tim_channel, freq, 0.50);
		} else if (freq < (-1 * MIN_FREQUENCY)) {
			if (dev->dir_mode == 0){
				HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, 1);
			}
			else if (dev->dir_mode == 1){
				HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, 0);
			}
			set_pwm(dev->timHandle, dev->tim_channel, (-1*freq), 0.50);
		} else {
			set_pwm(dev->timHandle, dev->tim_channel, 100, 1.0);
		}
		dev->freq = freq;
	}
}
