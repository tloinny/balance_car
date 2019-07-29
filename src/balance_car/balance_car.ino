#include <MPU6050.h>
#include <L298N.h>
#include <easy_pid.h>

const int ENA = 6;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 11;	/* only one motor */

L298N DCMotor(ENA,IN1,IN2,IN3,IN4,ENB);	/* motor driver */
PID_CONTROLLER PID_contorller(2, 0.7, 5, 0.00000001, 0.7);
MPU6050 mpu6050;

unsigned long timer = 0;
float timeStep = 0.008;

float pitch = 0;
float roll = 0;
float yaw = 0;

float output = 0;
int throttle = 0;

void setup()
{
	Serial.begin(115200);
	/* Initialize MPU6050 */
	while(!mpu6050.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
	  Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
	  delay(500);
	}
  Serial.println("ok");
	/* 
	 * Calibrate gyroscope. The calibration must be at rest.
	 * If you don't want calibrate, comment this line. 
	 */
	mpu6050.calibrateGyro();
	/*
	 * Set threshold sensivty. Default 3.
	 * If you don't want use threshold, comment this line or set 0.
	 */
	mpu6050.setThreshold(0.1);
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
	if (output < 0)
	{
		output = -1 * output;
		throttle = map(output, 0, 80, 20, 255);
		DCMotor.forward(throttle,0);
	}else if (output > 0)
	{
		throttle = map(output, 0, 80, 20, 255);
		DCMotor.backward(throttle,0);
	}else
	{
		DCMotor.full_stop(0);
	}
 Serial.print("YAxis:");
 Serial.print(norm.YAxis,DEC);
 Serial.print(" ");
	Serial.print("pitch:");
	Serial.print(pitch,DEC);
	Serial.print(" ");
	Serial.print("throttle:");
	Serial.println(throttle,DEC);
	delay((timeStep*1000) - (millis() - timer));
}
