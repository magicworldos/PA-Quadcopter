#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <if_mpu6050.h>

MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] =
{ '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

void mpu6050_setup()
{
	// initialize device
	printf("Initializing I2C devices...\n");
	mpu.initialize();

	// verify connection
	printf("Testing device connections...\n");
	printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

	// load and configure the DMP
	printf("Initializing DMP...\n");
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// turn on the DMP, now that it's ready
		printf("Enabling DMP...\n");
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		//Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		//attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		printf("DMP ready!\n");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		printf("DMP Initialization failed (code %d)\n", devStatus);
	}
}

void mpu6050_value(float *x, float *y, float *z, float *gx, float *gy, float *gz, float *ax, float *ay, float *az)
{
	// if programming failed, don't try to do anything
	if (!dmpReady)
	{
		return;
	}
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	if (fifoCount == 1024)
	{
		mpu.resetFIFO();
		printf("FIFO overflow!\n");

	}
	else if (fifoCount >= 42)
	{
		short aax, aay, aaz;
		short ggx, ggy, ggz;

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		*x = ypr[0] * 180 / M_PI;
		*y = ypr[1] * 180 / M_PI;
		*z = ypr[2] * 180 / M_PI;

		// display real acceleration, adjusted to remove gravity
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		*ax = (float)aaReal.x / 16384.0;
		*ay = (float)aaReal.y / 16384.0;
		*az = (float)aaReal.z / 16384.0;

		mpu.getRotation(&ggx, &ggy, &ggz);
		*gx = (float) (ggx) / 131.0;
		*gy = (float) (ggy) / 131.0;
		*gz = (float) (ggz) / 131.0;
	}
}

