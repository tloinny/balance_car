#include <MPU6050.h>
#include <L298N.h>
#include <easy_pid.h>

const int ENA = 0;
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;
const int ENB = 0;	/* only one motor */

L298N DCMotor(ENA,IN1,IN2,IN3,IN4,ENB);	/* motor driver */
PID_CONTROLLER PID_contorller(2, 0.7, 5, 0.00000001, 0.7);
MPU6050 mpu6050;

unsigned long timer = 0;
float timeStep = 0.01;

float pitch = 0;
float roll = 0;
float yaw = 0;

float output = 0;

void setup()
{
	Serial.begin(115200);
	/* Initialize MPU6050 */
	while(!mpu6050.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
	  Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
	  delay(500);
	}
	/* 
	 * Calibrate gyroscope. The calibration must be at rest.
	 * If you don't want calibrate, comment this line. 
	 */
	mpu6050.calibrateGyro();
	/*
	 * Set threshold sensivty. Default 3.
	 * If you don't want use threshold, comment this line or set 0.
	 */
	mpu6050.setThreshold(3);
	PID_contorller.setGoal(0);
}

void loop()
{
	timer = millis();
	/* Read normalized values */
	Vector norm = mpu6050.readNormalizeGyro();
	/* Calculate Pitch */
	pitch = pitch + norm.YAxis * timeStep;
	/* Wait to full timeStep period */
	output = PID_contorller.update(pitch);
	if (output > 0)
	{
		DCMotor.forward(output,0);
	}else if (output < 0)
	{
		DCMotor.backward(output,0);
	}else
	{
		DCMotor.full_stop(0);
	}
	delay((timeStep*1000) - (millis() - timer));
}
