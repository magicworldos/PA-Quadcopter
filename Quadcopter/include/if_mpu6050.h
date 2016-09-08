/*
 * if_mpu6050.h
 *
 *  Created on: Apr 26, 2016
 *      Author: lidq
 */

#ifndef IF_MPU6050_H_
#define IF_MPU6050_H_

extern "C"
{
	void mpu6050_setup();

	void mpu6050_value(float *x, float *y, float *z, float *gx, float *gy, float *gz);
}

#endif /* IF_MPU6050_H_ */
