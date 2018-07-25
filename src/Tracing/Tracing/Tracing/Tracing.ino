/*
 Name:		Tracing.ino
 Created:	7/24/2018 22:44:17 PM
 Author:	LamKamhang
 --------------------------------
 version:
	v1.0 - implement a simple tracing code. 
		   using PID algorithm.
*/

#include <Server.h>
#include <math.h>
#include "Gyro/Gyro.h"

/* some pins for the motor */
// may be better to use a const array.
#define _MOTOR_FL_PIN1_		3	// the front left motor's pin
#define _MOTOR_FL_PIN2_		4	// the front left motor's pin
#define _MOTOR_FR_PIN1_		5	
#define _MOTOR_FR_PIN2_		6	
#define _MOTOR_BL_PIN1_		7
#define _MOTOR_BL_PIN2_		8
#define _MOTOR_BR_PIN1_		9
#define _MOTOR_BR_PIN2_		10

/* some parameter for motor */
#define _FORWARD_SPEED_		200
#define _HIGH_THRESHOLD_	255
#define _LOW_THRESHOLD_		-255

/* some pins for the gray sensors */
const int _GRAY_IO_PIN_[] = { 1, 2, 3, 4, 5, 6, 7 };

/* some parameter for gray sensors */
#define _BLACK_THRESHOLD_	200
const float _GRAY_IO_WEIGHT_[] = { -120, -80, -40, 0, 40, 80, 120 };

/* PID algorithm's parameters */
#define _KP_	0.8
#define _KI_	0.0002
#define _KD_	50
#define _DT_	20

/* some pins for ultrasonic */
// may be better to use a const array.
#define _TRIG_PIN_	14
#define _ECHO_PIN_	15	

/* function header */
float calerror();
float caloutput(float curError);
void run(int param);

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	Gyro gyro(10);
}

// the loop function runs over and over again until power down or reset
void loop() {
	run(caloutput(calerror()));
}

float calerror()
{
	float error = 0;
	float sum = 7;
	for (int i = 0; i < 7; ++i)
	{
		// black line --> return 1
		// white --> return 0
		int value = 1 - digitalRead(_GRAY_IO_PIN_[i]);
		sum += value;
		error += value * _GRAY_IO_WEIGHT_[i];
	}

	// if the return value is small, means good.
	// if the return value is positive large, means turn left.
	// if the return value is negative large, means turn right.
	return error / sum;
}

float caloutput(float curError)
{
	static float preError = 0;
	static float integral = 0;

	integral += curError;
	float diff = (curError - preError);

	float output = _KP_ * curError +
		_KI_ * integral +
		_KD_ * diff;

	preError = curError;

	// adjust two motor's speed.
	// left motor minus it.
	// right motor plus it.
	return output;
}

void run(int param)
{
	// left motor
	int leftSpeed = _FORWARD_SPEED_ - param;
	if (leftSpeed < _LOW_THRESHOLD_)
		leftSpeed = _LOW_THRESHOLD_;
	else if (leftSpeed > _HIGH_THRESHOLD_)
		leftSpeed = _HIGH_THRESHOLD_;
	if (leftSpeed < 0)
	{
		analogWrite(_MOTOR_FL_PIN1_, 0);
		analogWrite(_MOTOR_FL_PIN2_, -leftSpeed);
		analogWrite(_MOTOR_BL_PIN1_, _FORWARD_SPEED_);
		analogWrite(_MOTOR_BL_PIN2_, 0);
	}
	else
	{
		analogWrite(_MOTOR_FL_PIN2_, 0);
		analogWrite(_MOTOR_FL_PIN1_, leftSpeed);
		analogWrite(_MOTOR_BL_PIN1_, _FORWARD_SPEED_);
		analogWrite(_MOTOR_BL_PIN2_, 0);
	}

	// right motor
	int rightSpeed = _FORWARD_SPEED_ + param;
	if (rightSpeed < _LOW_THRESHOLD_)
		rightSpeed = _LOW_THRESHOLD_;
	else if (rightSpeed > _HIGH_THRESHOLD_)
		rightSpeed = _HIGH_THRESHOLD_;
	if (rightSpeed < 0)
	{
		analogWrite(_MOTOR_FR_PIN1_, 0);
		analogWrite(_MOTOR_FR_PIN2_, -rightSpeed);
		analogWrite(_MOTOR_BR_PIN1_, _FORWARD_SPEED_);
		analogWrite(_MOTOR_BR_PIN2_, 0);
	}
	else
	{
		analogWrite(_MOTOR_FR_PIN2_, 0);
		analogWrite(_MOTOR_FR_PIN1_, rightSpeed);
		analogWrite(_MOTOR_BR_PIN1_, _FORWARD_SPEED_);
		analogWrite(_MOTOR_BR_PIN2_, 0);
	}
}


//超声模块测距
float ultrasonicCheck()
{
	digitalWrite(_TRIG_PIN_, LOW);
	delayMicroseconds(2);
	digitalWrite(_TRIG_PIN_, HIGH);
	delayMicroseconds(10);
	digitalWrite(_TRIG_PIN_, LOW);
	return pulseIn(_ECHO_PIN_, HIGH) / 58.00;
}

