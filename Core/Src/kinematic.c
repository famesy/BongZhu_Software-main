/*
 * kinematic.c
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#include "kinematic.h"

uint8_t IPK(float x, float y, float z, float pitch, float roll,
		float *config_arr) {
	/*
	 IPK does calculate configuration of robot (joint space) from task space configuration.

	 :param x, y, z, pitch, roll is task space variable.
	 :param config_arr is output of
	 :return: 0 if it's not in workspace or something i doesn't know lol.
	 :return: 1 if it's can calculate a value.
	 */
	float q_arr[5] = { 0 };
	const float l1 = 236.0f, l2 = 352.278299, l3 = 410.487515, l5 = 146.0f,
			l3_ = 60.0f;
	const float k1 = 0.113792, k3 = 0.048741;
	float p12 = 2.469986;
	float r = sqrt(pow(x, 2) + pow(y, 2));
	float r_ = r - l5 * cos(-pitch);
	float z_ = z - l1 - l5 * sin(-pitch);
	float dis = sqrt(pow(r_, 2) + pow(z_, 2));
	if (dis < 72.1 || dis > 822) {
		return 0;
	}
	q_arr[0] = atan2(y, x);

	float rSam = 0;
	float q3_min = 0;
	float q3_max = PI / 2;
	float dx = 0;
	float dy = 0;
	while (fabs(rSam - dis) > 0.01) {
		q_arr[2] = (q3_max + q3_min) / 2;
		dy = -l3 * cos(q_arr[2] + k3) + l2 * cos(q_arr[2] - k1);
		dx = l3_ + l3 * sin(q_arr[2] + k3) + l2 * sin(q_arr[2] - k1);
		rSam = sqrt(pow(dx, 2) + pow(dy, 2));
		if (rSam > dis) {
			q3_max = q_arr[2];
		} else {
			q3_min = q_arr[2];
		}
	}
	float Q324 = PI / 2 - q_arr[2] + k1 - atan2(dy, dx);
	float Qperk = atan(z_ / r_);
	q_arr[1] = Qperk + Q324 - p12;
	if (q_arr[1] < -2.35619449019 || q_arr[1] > 0) {
		return 0;
	}
	float Q342 = PI - (k3 - k1 + 2 * q_arr[2]) - Q324;
	q_arr[3] = (-pitch - Qperk + Q342
			- (PI / 4 - 0.048741851309931590452344789123));
	q_arr[4] = roll;
	memcpy(config_arr, q_arr, sizeof(q_arr));
	return 1;
}

void IVK(float q[5], float x_dot[5], float *m_dot) {
	/*
	 IVK does calculate inverse velocity kinematic.

	 :param q[0-4]: configuration (task space) of robot from j0-4
	 :param x_dot[0-4]: configuration (task space) of robot from j0-4
	 :return: NONE
	 */
	float m_dot_temp[5];
	double dv[25];
	  double J_inv_tmp;
	  double J_inv_tmp_tmp;
	  double b_J_inv_tmp;
	  double b_J_inv_tmp_tmp;
	  double c_J_inv_tmp;
	  double c_J_inv_tmp_tmp;
	  double d;
	  double d1;
	  double d10;
	  double d11;
	  double d12;
	  double d13;
	  double d14;
	  double d15;
	  double d16;
	  double d17;
	  double d2;
	  double d3;
	  double d4;
	  double d5;
	  double d6;
	  double d7;
	  double d8;
	  double d9;
	  double d_J_inv_tmp;
	  double d_J_inv_tmp_tmp;
	  double e_J_inv_tmp;
	  double e_J_inv_tmp_tmp;
	  double f_J_inv_tmp_tmp;
	  int i;
	  int i1;
	  J_inv_tmp_tmp = q[1] + 2.0 * q[2];
	  b_J_inv_tmp_tmp =
	      ((((J_inv_tmp_tmp + q[3]) + 1.6845) + -0.8995) + -0.73665) + 1.52205;
	  c_J_inv_tmp_tmp = sin(b_J_inv_tmp_tmp);
	  d_J_inv_tmp_tmp = sin(q[0]);
	  J_inv_tmp = cos(((J_inv_tmp_tmp + 1.6845) + -0.8995) + -0.73665);
	  b_J_inv_tmp = cos(b_J_inv_tmp_tmp);
	  J_inv_tmp_tmp = ((q[1] + q[2]) + 1.6845) + -0.8995;
	  c_J_inv_tmp = cos(J_inv_tmp_tmp);
	  d_J_inv_tmp = sin(J_inv_tmp_tmp);
	  J_inv_tmp_tmp = sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665);
	  b_J_inv_tmp_tmp = cos(q[1] + 1.6845);
	  e_J_inv_tmp_tmp = sin(q[1] + 1.6845);
	  f_J_inv_tmp_tmp = cos(q[0]);
	  e_J_inv_tmp = cos(q[2]);
	  dv[0] = 0.0;
	  dv[5] = 0.0;
	  d = f_J_inv_tmp_tmp * f_J_inv_tmp_tmp;
	  d1 = d_J_inv_tmp_tmp * d_J_inv_tmp_tmp;
	  dv[10] =
	      -(25.0 * d_J_inv_tmp_tmp) /
	      (9.0 *
	       (((((((146.0 * c_J_inv_tmp_tmp * d + 410.488 * J_inv_tmp * d) +
	             146.0 *
	                 sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                      -0.73665) +
	                     1.52205) *
	                 d1) +
	            410.488 *
	                cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                d1) +
	           60.0 * c_J_inv_tmp * d) +
	          60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) * d1) +
	         352.278 * b_J_inv_tmp_tmp * d) +
	        352.278 * cos(q[1] + 1.6845) * d1));
	  dv[15] =
	      25.0 * f_J_inv_tmp_tmp /
	      (9.0 *
	       (((((((146.0 *
	                  sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                       -0.73665) +
	                      1.52205) *
	                  (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp) +
	              410.488 *
	                  cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                  (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	             146.0 *
	                 sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                      -0.73665) +
	                     1.52205) *
	                 (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	            410.488 *
	                cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	           60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) *
	               (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	          60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) *
	              (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	         352.278 * cos(q[1] + 1.6845) * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	        352.278 * cos(q[1] + 1.6845) * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)));
	  dv[20] = 0.0;
	  d2 = sin(((q[1] + q[2]) + 1.6845) + -0.8995);
	  d3 = 24629.28 * cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	       d2;
	  d4 = 24629.28 * J_inv_tmp_tmp * cos(((q[1] + q[2]) + 1.6845) + -0.8995);
	  d5 = 289211.783328 *
	       cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	       e_J_inv_tmp_tmp;
	  d6 = 289211.783328 * J_inv_tmp_tmp * b_J_inv_tmp_tmp;
	  d7 = 21136.68 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) * e_J_inv_tmp_tmp;
	  d8 = 21136.68 * d2 * b_J_inv_tmp_tmp;
	  dv[1] = 27.0 *
	          (((8760.0 * c_J_inv_tmp_tmp * d_J_inv_tmp * d_J_inv_tmp_tmp +
	             119862.496 * b_J_inv_tmp * J_inv_tmp * d_J_inv_tmp_tmp) +
	            119862.496 * c_J_inv_tmp_tmp * J_inv_tmp_tmp * d_J_inv_tmp_tmp) +
	           8760.0 * b_J_inv_tmp * c_J_inv_tmp * d_J_inv_tmp_tmp) /
	          (((((((((((24629.28 * J_inv_tmp * d_J_inv_tmp * d -
	                     24629.28 * J_inv_tmp_tmp * c_J_inv_tmp * d) +
	                    d3 * d1) -
	                   d4 * d1) +
	                  289211.783328 * J_inv_tmp * e_J_inv_tmp_tmp * d) -
	                 d6 * d) +
	                d5 * d1) -
	               d6 * d1) +
	              21136.68 * c_J_inv_tmp * e_J_inv_tmp_tmp * d) -
	             21136.68 * d_J_inv_tmp * b_J_inv_tmp_tmp * d) +
	            d7 * d1) -
	           d8 * d1);
	  d9 = 8760.0 *
	       cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) + -0.73665) +
	           1.52205) *
	       cos(((q[1] + q[2]) + 1.6845) + -0.8995);
	  d10 = 8760.0 *
	        sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) + -0.73665) +
	            1.52205) *
	        sin(((q[1] + q[2]) + 1.6845) + -0.8995);
	  d11 = ((((((((((d3 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp) -
	                  d4 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                 d3 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                d4 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	               d5 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	              d6 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	             d5 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	            d6 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	           d7 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	          d8 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	         d7 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	        d8 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp);
	  dv[6] = -(27.0 *
	            (((d10 * f_J_inv_tmp_tmp +
	               119862.496 *
	                   cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                        -0.73665) +
	                       1.52205) *
	                   cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                   f_J_inv_tmp_tmp) +
	              119862.496 *
	                  sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                       -0.73665) +
	                      1.52205) *
	                  sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                  f_J_inv_tmp_tmp) +
	             d9 * f_J_inv_tmp_tmp)) /
	          d11;
	  dv[11] = -(27.0 * (820.976 * J_inv_tmp * f_J_inv_tmp_tmp +
	                     60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                         f_J_inv_tmp_tmp)) /
	           d11;
	  dv[16] =
	      -(27.0 *
	        (820.976 * cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	             d_J_inv_tmp_tmp +
	         60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) * d_J_inv_tmp_tmp)) /
	      d11;
	  dv[21] = -(27.0 * (820.976 * J_inv_tmp_tmp + 60.0 * d_J_inv_tmp)) / d11;
	  d12 = 2.0 * d11;
	  dv[2] = -(55.0 * (((((59931.248 * b_J_inv_tmp * J_inv_tmp * e_J_inv_tmp *
	                            d_J_inv_tmp_tmp +
	                        59931.248 * c_J_inv_tmp_tmp * J_inv_tmp_tmp *
	                            e_J_inv_tmp * d_J_inv_tmp_tmp) +
	                       d9 * e_J_inv_tmp * d_J_inv_tmp_tmp) +
	                      51432.588 * b_J_inv_tmp * b_J_inv_tmp_tmp * e_J_inv_tmp *
	                          d_J_inv_tmp_tmp) +
	                     d10 * e_J_inv_tmp * d_J_inv_tmp_tmp) +
	                    51432.588 * c_J_inv_tmp_tmp * e_J_inv_tmp_tmp *
	                        e_J_inv_tmp * d_J_inv_tmp_tmp)) /
	          d12;
	  dv[7] =
	      55.0 *
	      (((((59931.248 *
	               cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                    -0.73665) +
	                   1.52205) *
	               cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	               f_J_inv_tmp_tmp * e_J_inv_tmp +
	           59931.248 *
	               sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                    -0.73665) +
	                   1.52205) *
	               sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	               f_J_inv_tmp_tmp * e_J_inv_tmp) +
	          8760.0 *
	              cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                   -0.73665) +
	                  1.52205) *
	              cos(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[0]) *
	              e_J_inv_tmp) +
	         51432.588 *
	             cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                  -0.73665) +
	                 1.52205) *
	             cos(q[1] + 1.6845) * f_J_inv_tmp_tmp * e_J_inv_tmp) +
	        8760.0 *
	            sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                 -0.73665) +
	                1.52205) *
	            sin(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[0]) * e_J_inv_tmp) +
	       51432.588 *
	           sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                -0.73665) +
	               1.52205) *
	           sin(q[1] + 1.6845) * f_J_inv_tmp_tmp * e_J_inv_tmp) /
	      d12;
	  dv[12] =
	      55.0 *
	      ((60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[0]) *
	            e_J_inv_tmp +
	        352.278 * cos(q[1] + 1.6845) * f_J_inv_tmp_tmp * e_J_inv_tmp) +
	       410.488 * cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	           f_J_inv_tmp_tmp * e_J_inv_tmp) /
	      d12;
	  dv[17] =
	      55.0 *
	      ((60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) * e_J_inv_tmp *
	            d_J_inv_tmp_tmp +
	        352.278 * cos(q[1] + 1.6845) * e_J_inv_tmp * d_J_inv_tmp_tmp) +
	       410.488 * cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	           e_J_inv_tmp * d_J_inv_tmp_tmp) /
	      d12;
	  dv[22] = 55.0 *
	           ((410.488 * J_inv_tmp_tmp * e_J_inv_tmp +
	             60.0 * sin(((q[1] + q[2]) + 1.6845) + -0.8995) * e_J_inv_tmp) +
	            352.278 * e_J_inv_tmp_tmp * e_J_inv_tmp) /
	           d12;
	  d9 = 21136.68 *
	       sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) + -0.73665) +
	           1.52205);
	  d10 = d9 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) * e_J_inv_tmp_tmp;
	  d9 = d9 * d2 * b_J_inv_tmp_tmp;
	  d12 = 24629.28 *
	        sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) + -0.73665) +
	            1.52205);
	  d2 *= d12 * cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665);
	  d12 = d12 * J_inv_tmp_tmp * cos(((q[1] + q[2]) + 1.6845) + -0.8995);
	  e_J_inv_tmp =
	      289211.783328 *
	      sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) + -0.73665) +
	          1.52205);
	  d13 = e_J_inv_tmp *
	        cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	        e_J_inv_tmp_tmp;
	  e_J_inv_tmp = e_J_inv_tmp * J_inv_tmp_tmp * b_J_inv_tmp_tmp;
	  d14 = c_J_inv_tmp_tmp * c_J_inv_tmp_tmp;
	  d15 = 87600.0 * d14 * d_J_inv_tmp;
	  d14 = 1.02865176E+6 * d14 * e_J_inv_tmp_tmp;
	  d16 = d15 * d_J_inv_tmp_tmp;
	  d17 = d14 * d_J_inv_tmp_tmp;
	  dv[3] =
	      4.0 *
	      (((((((((((((((221663.52 * J_inv_tmp * d_J_inv_tmp * f_J_inv_tmp_tmp -
	                     221663.52 * J_inv_tmp_tmp * c_J_inv_tmp *
	                         f_J_inv_tmp_tmp) +
	                    2.6029060499520004E+6 * J_inv_tmp * e_J_inv_tmp_tmp *
	                        f_J_inv_tmp_tmp) -
	                   2.6029060499520004E+6 * J_inv_tmp_tmp * b_J_inv_tmp_tmp *
	                       f_J_inv_tmp_tmp) +
	                  190230.12000000002 * c_J_inv_tmp * e_J_inv_tmp_tmp *
	                      f_J_inv_tmp_tmp) -
	                 190230.12000000002 * d_J_inv_tmp * b_J_inv_tmp_tmp *
	                     f_J_inv_tmp_tmp) +
	                d16) +
	               d17) +
	              87600.0 * b_J_inv_tmp * c_J_inv_tmp_tmp * c_J_inv_tmp *
	                  d_J_inv_tmp_tmp) +
	             1.02865176E+6 * b_J_inv_tmp * c_J_inv_tmp_tmp * b_J_inv_tmp_tmp *
	                 d_J_inv_tmp_tmp) +
	            246292.8 * c_J_inv_tmp_tmp * J_inv_tmp * d_J_inv_tmp *
	                d_J_inv_tmp_tmp) -
	           246292.8 *
	               sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                    -0.73665) +
	                   1.52205) *
	               J_inv_tmp_tmp * c_J_inv_tmp * d_J_inv_tmp_tmp) +
	          2.8921178332800004E+6 * c_J_inv_tmp_tmp * J_inv_tmp *
	              e_J_inv_tmp_tmp * d_J_inv_tmp_tmp) -
	         2.8921178332800004E+6 *
	             sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                  -0.73665) +
	                 1.52205) *
	             J_inv_tmp_tmp * b_J_inv_tmp_tmp * d_J_inv_tmp_tmp) +
	        211366.80000000002 * c_J_inv_tmp_tmp * c_J_inv_tmp * e_J_inv_tmp_tmp *
	            d_J_inv_tmp_tmp) -
	       211366.80000000002 *
	           sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                -0.73665) +
	               1.52205) *
	           d_J_inv_tmp * b_J_inv_tmp_tmp * d_J_inv_tmp_tmp) /
	      (9.0 *
	       (((((((((((21136.68 * c_J_inv_tmp_tmp * c_J_inv_tmp * e_J_inv_tmp_tmp *
	                      d -
	                  21136.68 *
	                      sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                           -0.73665) +
	                          1.52205) *
	                      d_J_inv_tmp * b_J_inv_tmp_tmp * d) +
	                 d10 * d1) -
	                d9 * d1) +
	               24629.28 * c_J_inv_tmp_tmp * J_inv_tmp * d_J_inv_tmp * d) -
	              24629.28 *
	                  sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                       -0.73665) +
	                      1.52205) *
	                  J_inv_tmp_tmp * c_J_inv_tmp * d) +
	             d2 * d1) -
	            d12 * d1) +
	           289211.783328 * c_J_inv_tmp_tmp * J_inv_tmp * e_J_inv_tmp_tmp * d) -
	          e_J_inv_tmp * d) +
	         d13 * d1) -
	        e_J_inv_tmp * d1));
	  d = 9.0 * (((((((((((d10 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp) -
	                       d9 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                      d10 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                     d9 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	                    d2 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	                   d12 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                  d2 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                 d12 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	                d13 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	               e_J_inv_tmp * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	              d13 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	             e_J_inv_tmp * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp));
	  d1 = d15 * f_J_inv_tmp_tmp;
	  d2 = d14 * f_J_inv_tmp_tmp;
	  dv[8] =
	      -(4.0 *
	        (((((((((((((((221663.52 *
	                           sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                               -0.73665) *
	                           cos(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                           d_J_inv_tmp_tmp -
	                       221663.52 *
	                           cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                               -0.73665) *
	                           sin(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                           d_J_inv_tmp_tmp) -
	                      2.6029060499520004E+6 *
	                          cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                              -0.73665) *
	                          sin(q[1] + 1.6845) * d_J_inv_tmp_tmp) +
	                     2.6029060499520004E+6 *
	                         sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                             -0.73665) *
	                         cos(q[1] + 1.6845) * d_J_inv_tmp_tmp) +
	                    d1) -
	                   190230.12000000002 *
	                       cos(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                       sin(q[1] + 1.6845) * d_J_inv_tmp_tmp) +
	                  190230.12000000002 * sin(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                      cos(q[1] + 1.6845) * d_J_inv_tmp_tmp) +
	                 d2) +
	                87600.0 *
	                    cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                         -0.73665) +
	                        1.52205) *
	                    sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                         -0.73665) +
	                        1.52205) *
	                    cos(((q[1] + q[2]) + 1.6845) + -0.8995) * f_J_inv_tmp_tmp) +
	               1.02865176E+6 *
	                   cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                        -0.73665) +
	                       1.52205) *
	                   sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                        -0.73665) +
	                       1.52205) *
	                   cos(q[1] + 1.6845) * f_J_inv_tmp_tmp) +
	              246292.8 *
	                  sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                       -0.73665) +
	                      1.52205) *
	                  cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                  sin(((q[1] + q[2]) + 1.6845) + -0.8995) * f_J_inv_tmp_tmp) -
	             246292.8 *
	                 sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                      -0.73665) +
	                     1.52205) *
	                 sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                 cos(((q[1] + q[2]) + 1.6845) + -0.8995) * f_J_inv_tmp_tmp) +
	            2.8921178332800004E+6 *
	                sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                     -0.73665) +
	                    1.52205) *
	                cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                sin(q[1] + 1.6845) * f_J_inv_tmp_tmp) -
	           2.8921178332800004E+6 *
	               sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                    -0.73665) +
	                   1.52205) *
	               sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	               cos(q[1] + 1.6845) * f_J_inv_tmp_tmp) +
	          211366.80000000002 *
	              sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                   -0.73665) +
	                  1.52205) *
	              cos(((q[1] + q[2]) + 1.6845) + -0.8995) * sin(q[1] + 1.6845) *
	              f_J_inv_tmp_tmp) -
	         211366.80000000002 *
	             sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                  -0.73665) +
	                 1.52205) *
	             sin(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[1] + 1.6845) *
	             f_J_inv_tmp_tmp)) /
	      d;
	  d9 = 9.0 * d11;
	  dv[13] = -(40.0 * f_J_inv_tmp_tmp *
	             (60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) +
	              704.556 * b_J_inv_tmp_tmp)) /
	           d9;
	  dv[18] =
	      -(40.0 * (60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) * sin(q[0]) +
	                704.556 * cos(q[1] + 1.6845) * d_J_inv_tmp_tmp)) /
	      d9;
	  dv[23] = -(40.0 * (60.0 * sin(((q[1] + q[2]) + 1.6845) + -0.8995) +
	                     704.556 * e_J_inv_tmp_tmp)) /
	           d9;
	  dv[4] =
	      4.0 *
	      (((((((((((((((221663.52 *
	                         sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                             -0.73665) *
	                         cos(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[0]) -
	                     221663.52 *
	                         cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                             -0.73665) *
	                         sin(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[0])) -
	                    2.6029060499520004E+6 *
	                        cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                            -0.73665) *
	                        sin(q[1] + 1.6845) * cos(q[0])) +
	                   2.6029060499520004E+6 *
	                       sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                           -0.73665) *
	                       cos(q[1] + 1.6845) * cos(q[0])) -
	                  190230.12000000002 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                      sin(q[1] + 1.6845) * cos(q[0])) +
	                 190230.12000000002 * sin(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                     cos(q[1] + 1.6845) * cos(q[0])) +
	                d16) +
	               d17) +
	              87600.0 *
	                  cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                       -0.73665) +
	                      1.52205) *
	                  sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                       -0.73665) +
	                      1.52205) *
	                  cos(((q[1] + q[2]) + 1.6845) + -0.8995) * sin(q[0])) +
	             1.02865176E+6 *
	                 cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                      -0.73665) +
	                     1.52205) *
	                 sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                      -0.73665) +
	                     1.52205) *
	                 cos(q[1] + 1.6845) * sin(q[0])) +
	            246292.8 *
	                sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                     -0.73665) +
	                    1.52205) *
	                cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                sin(((q[1] + q[2]) + 1.6845) + -0.8995) * sin(q[0])) -
	           246292.8 *
	               sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                    -0.73665) +
	                   1.52205) *
	               sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	               cos(((q[1] + q[2]) + 1.6845) + -0.8995) * sin(q[0])) +
	          2.8921178332800004E+6 *
	              sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                   -0.73665) +
	                  1.52205) *
	              cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	              sin(q[1] + 1.6845) * sin(q[0])) -
	         2.8921178332800004E+6 *
	             sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                  -0.73665) +
	                 1.52205) *
	             sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	             cos(q[1] + 1.6845) * sin(q[0])) +
	        211366.80000000002 *
	            sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                 -0.73665) +
	                1.52205) *
	            cos(((q[1] + q[2]) + 1.6845) + -0.8995) * sin(q[1] + 1.6845) *
	            sin(q[0])) -
	       211366.80000000002 *
	           sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                -0.73665) +
	               1.52205) *
	           sin(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[1] + 1.6845) *
	           sin(q[0])) /
	      d;
	  dv[9] =
	      -(4.0 *
	        (((((((((((((((221663.52 *
	                           cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                               -0.73665) *
	                           sin(((q[1] + q[2]) + 1.6845) + -0.8995) * sin(q[0]) -
	                       221663.52 *
	                           sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                               -0.73665) *
	                           cos(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                           sin(q[0])) +
	                      2.6029060499520004E+6 *
	                          cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                              -0.73665) *
	                          sin(q[1] + 1.6845) * sin(q[0])) -
	                     2.6029060499520004E+6 *
	                         sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) +
	                             -0.73665) *
	                         cos(q[1] + 1.6845) * sin(q[0])) +
	                    d1) +
	                   190230.12000000002 *
	                       cos(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                       sin(q[1] + 1.6845) * sin(q[0])) -
	                  190230.12000000002 * sin(((q[1] + q[2]) + 1.6845) + -0.8995) *
	                      cos(q[1] + 1.6845) * sin(q[0])) +
	                 d2) +
	                87600.0 *
	                    cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                         -0.73665) +
	                        1.52205) *
	                    sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                         -0.73665) +
	                        1.52205) *
	                    cos(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[0])) +
	               1.02865176E+6 *
	                   cos((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                        -0.73665) +
	                       1.52205) *
	                   sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                        -0.73665) +
	                       1.52205) *
	                   cos(q[1] + 1.6845) * cos(q[0])) +
	              246292.8 *
	                  sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                       -0.73665) +
	                      1.52205) *
	                  cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                  sin(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[0])) -
	             246292.8 *
	                 sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                      -0.73665) +
	                     1.52205) *
	                 sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                 cos(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[0])) +
	            2.8921178332800004E+6 *
	                sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                     -0.73665) +
	                    1.52205) *
	                cos((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	                sin(q[1] + 1.6845) * cos(q[0])) -
	           2.8921178332800004E+6 *
	               sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                    -0.73665) +
	                   1.52205) *
	               sin((((q[1] + 2.0 * q[2]) + 1.6845) + -0.8995) + -0.73665) *
	               cos(q[1] + 1.6845) * cos(q[0])) +
	          211366.80000000002 *
	              sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                   -0.73665) +
	                  1.52205) *
	              cos(((q[1] + q[2]) + 1.6845) + -0.8995) * sin(q[1] + 1.6845) *
	              cos(q[0])) -
	         211366.80000000002 *
	             sin((((((q[1] + 2.0 * q[2]) + q[3]) + 1.6845) + -0.8995) +
	                  -0.73665) +
	                 1.52205) *
	             sin(((q[1] + q[2]) + 1.6845) + -0.8995) * cos(q[1] + 1.6845) *
	             cos(q[0]))) /
	      d;
	  dv[14] = -(40.0 * cos(q[0]) *
	             (60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) +
	              704.556 * cos(q[1] + 1.6845))) /
	           (9.0 * (((((((((((d3 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp) -
	                             d4 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                            d3 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                           d4 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	                          d5 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	                         d6 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                        d5 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                       d6 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	                      d7 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	                     d8 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                    d7 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                   d8 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)));
	  dv[19] =
	      -(40.0 * (60.0 * cos(((q[1] + q[2]) + 1.6845) + -0.8995) * sin(q[0]) +
	                704.556 * cos(q[1] + 1.6845) * sin(q[0]))) /
	      (9.0 * (((((((((((d3 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp) -
	                        d4 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                       d3 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                      d4 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	                     d5 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	                    d6 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                   d5 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                  d6 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	                 d7 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	                d8 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	               d7 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	              d8 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)));
	  dv[24] = -(40.0 * (60.0 * sin(((q[1] + q[2]) + 1.6845) + -0.8995) +
	                     704.556 * sin(q[1] + 1.6845))) /
	           (9.0 * (((((((((((d3 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp) -
	                             d4 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                            d3 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                           d4 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	                          d5 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	                         d6 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                        d5 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                       d6 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) +
	                      d7 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) -
	                     d8 * (f_J_inv_tmp_tmp * f_J_inv_tmp_tmp)) +
	                    d7 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)) -
	                   d8 * (d_J_inv_tmp_tmp * d_J_inv_tmp_tmp)));
	  for (i = 0; i < 5; i++) {
	    d = 0.0;
	    for (i1 = 0; i1 < 5; i1++) {
	      d += dv[i + 5 * i1] * x_dot[i1];
	    }
	    m_dot_temp[i] = d;
	  }
	memcpy(m_dot, m_dot_temp, 20);
}
