/*
 * KalmanFilter.h
 *
 *  Created on: Jan 28, 2022
 *      Author: fame
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_


#define DT 0.002

typedef struct {
	/* Parameter */
	float R;
	float Q;

	/* KF "memory" */
	float x1;
	float x2;
	float p11;
	float p12;
	float p21;
	float p22;
} KalmanFilter;

void KalmanFilter_initialise(KalmanFilter *dev, float x1,float x2,float p11,float p12,float p21,float p22, float R, float Q);
void KalmanFilter_Update(KalmanFilter *dev,float theta_k);
#endif /* INC_KALMANFILTER_H_ */
