#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <if_mpu6050.h>

MPU6050 mpu;
// MPU control/status vars
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
int16_t gyro[3];    	// [x, y, z]            gyro vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] =
{ '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
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

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

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
		// reset so we can continue cleanly
		mpu.resetFIFO();
		printf("FIFO overflow!\n");

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else

	if (fifoCount >= 42)
	{
		//printf("%3.3f\t%3.3f\n", gravity.x, *gx);

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		*x = (float) (ypr[0] * 180.0 / M_PI);
		*y = (float) (ypr[1] * 180.0 / M_PI);
		*z = (float) (ypr[2] * 180.0 / M_PI);
		//printf("ypr  %7.2f %7.2f %7.2f\n", ypr[0] * 180 / M_PI, ypr[1] * 180 / M_PI, ypr[2] * 180 / M_PI);

		short aax, aay, aaz;
		short ggx, ggy, ggz;

		mpu.getRotation(&ggx, &ggy, &ggz);
		*gx = (float) (ggx) / 131.0;
		*gy = (float) (ggy) / 131.0;
		*gz = (float) (ggz) / 131.0;

	//	mpu.getAcceleration(&aax, &aay, &aaz);
	//	*ax = (float) (aax) / 131.0;
	//	*ay = (float) (aay) / 131.0;
	//	*az = (float) (aaz) / 131.0;

	}
}
