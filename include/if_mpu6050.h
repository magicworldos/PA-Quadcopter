/*
 * if_mpu6050.h
 *
 *  Created on: Apr 26, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef IF_MPU6050_H_
#define IF_MPU6050_H_

extern "C"
{
	void mpu6050_setup();

	void mpu6050_value(float *x, float *y, float *z, float *gx, float *gy, float *gz, float *ax, float *ay, float *az);
}

#endif /* IF_MPU6050_H_ */
