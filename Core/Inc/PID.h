/*
 * PID.h
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <KalmanFilter.h>

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Output limits */
	float lim_min;
	float lim_max;

	/* Integrator limits */
	float lim_int_max;
	float lim_int_min;

	/* Controller "memory" */
	float proportional_term;
	float integrator;
	float integral_term;
	float derivative_term;
	float prevError;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

void PIDController_initialise(PIDController *pid, float Kp, float Ki, float Kd);
float PIDController_update(PIDController *pid, float setpoint, float measurement);
float Cascade_PIDController_update(PIDController *position_pid,
		PIDController *velocity_pid, KalmanFilter *kalman_filter, float desired_position,
		float desired_velocity);
#endif /* INC_PID_H_ */
