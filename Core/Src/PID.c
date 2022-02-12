/*
 * PID.c
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */
#include "PID.h"

void PIDController_initialise(PIDController *pid, float Kp, float Ki, float Kd){
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->out = 0;
}

float PIDController_update(PIDController *pid, float setpoint, float measurement) {
	float error = setpoint - measurement;
	/*
	 * P term
	 */
	pid->proportional_term = pid->Kp * error;

	/*
	 * I term
	 */
	pid->integrator += error;
	pid->integral_term = pid->Ki * pid->integrator;
	/*
	 * D term
	 */
	pid->derivative_term = pid->Kd * (error - pid->prevError);
	pid->prevError = error;
	/*
	 * Calculate a final value
	 */
	pid->out = pid->proportional_term + pid->integral_term + pid->derivative_term;
	if (pid->out > 2500){
		pid->out = 2500;
	}
	else if(pid->out < -2500){
		pid->out = -2500;
	}
	return pid->out;
}

float Cascade_PIDController_update(PIDController *position_pid,
		PIDController *velocity_pid, KalmanFilter *kalman_filter, float desired_position,
		float desired_velocity) {
	float velocity_command = PIDController_update(position_pid, desired_position, kalman_filter->x1);
	float velocity_error = velocity_command + desired_velocity - kalman_filter->x2;
	float out = PIDController_update(velocity_pid, velocity_error, kalman_filter->x2);
	return out;
}
