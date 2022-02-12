/*
 * kinematic.h
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#ifndef INC_KINEMATIC_H_
#define INC_KINEMATIC_H_

/*
 * Include
 */
#include "math.h"
#include "stdint.h"
#include "string.h" //for memcpy

/*
 * DEFINES
 */
#define PI 3.142857

/*
 * STRUCT
 */

/*
 * FUNCTIONS
 */
uint8_t IPK(float x, float y, float z, float pitch, float roll, float *config_arr);
void IVK(float q[5], float x_dot[5], float *m_dot);


#endif /* INC_KINEMATIC_H_ */
