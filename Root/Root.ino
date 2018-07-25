#include <Wire.h>
#include "MPU6050.h"
#include"Gyro.h"

//定义超声波探测引脚
const int leftTrigPin = 23;
const int leftEchoPin = 22;
const int rightTrigPin = 44;
const int rightEchoPin = 45;		
//定义三路灰度传感器
const int grey_a = 19;
const int grey_b = 18;
const int grey_c = 17;					
//定义电机管脚
const int left_front_wheel1 = 4;
const int left_front_wheel2 = 5;
const int right_front_wheel1 = 10;
const int right_front_wheel2 = 11;
const int left_behind_wheel1 = 9;
const int left_behind_wheel2 = 8;
const int right_behind_wheel1 = 7;
const int right_behind_wheel2 = 6;
//定义七路灰度传感器
const int _GRAY_IO_PIN_[] = { 30, 32, 34, 36, 38, 40, 42 };
const float _GRAY_IO_WEIGHT_[] = { -120, -80, -40, 0, 40, 80, 120 };

Gyro gyro(2);
float basic_velocity;		//定义正常速度
float current_velocity;		//定义当前速度
float distance;				//定义最小距离变量
int near_Barrier = 0;			//定义避障状态变量
int slow_down = 0;			//定义减速状态

void barrier()
{
	digitalWrite(leftTrigPin, LOW);
	delay(2);
	digitalWrite(leftTrigPin, HIGH);
	delay(10);
	digitalWrite(leftTrigPin, LOW);
	distance = pulseIn(leftEchoPin, HIGH,6000)*0.017;
	digitalWrite(rightTrigPin, LOW);
	delay(2);
	digitalWrite(rightTrigPin, HIGH);
	delay(10);
	digitalWrite(rightTrigPin, LOW);
	distance = pulseIn(rightEchoPin, HIGH,6000)*0.017;
	if (near_Barrier == 1 && distance < 20 && distance>5)
		avoid_Barrier();
}

float calerror()
{
	float error = 0;
	float sum = 0;
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

void verify()
{
	float delta = calerror();
	delta *= 1;
	Serial.print("delta:");
	Serial.println(delta);
	gyro.setInitYaw(delta);
}

void motor(float e)
{
	float slow_velocity;
	int left_velocity, right_velocity;
	float kc = 10;
	slow_velocity = current_velocity - kc * e;
	slow_velocity = slow_velocity > 0 ? slow_velocity : 0;
	slow_velocity = slow_velocity < 255 ? slow_velocity : 255;
	Serial.print("slow:");
	Serial.println(slow_velocity);
	if (e > 0)
	{
		left_velocity = (int)current_velocity;
		right_velocity = (int)slow_velocity;
	}
	else
	{
		left_velocity = (int)slow_velocity;
		right_velocity = (int)current_velocity;
	}
	digitalWrite(left_front_wheel2, LOW);
	analogWrite(left_front_wheel1, left_velocity);	//前左轮正转
	digitalWrite(left_behind_wheel1, LOW);
	analogWrite(left_behind_wheel2, left_velocity);	//后左轮正转
	digitalWrite(right_front_wheel2, LOW);
	analogWrite(right_front_wheel2, right_velocity);//前右轮正转
	digitalWrite(right_behind_wheel2, LOW);
	analogWrite(right_behind_wheel1, right_velocity);//后右轮正转
}

void avoid_Barrier()
{

	near_Barrier = 0;
}

void interrupt()
{
	detachInterrupt(5);
	attachInterrupt(4, slow, RISING);		//利用19号口作为中断
	attachInterrupt(5, stop, FALLING);		//利用18号口作为中断
}

void slow()
{
	slow_down = 1;
	current_velocity = 50;
}

void stop()
{
	current_velocity = 0;
	motor(0);
	delay(5000);
	current_velocity = basic_velocity;
}


void setup() {
	Wire.begin();
	Serial.begin(9600);
	basic_velocity = 100;
	current_velocity = basic_velocity;
	pinMode(leftTrigPin, OUTPUT);
	pinMode(rightTrigPin, OUTPUT);
	pinMode(leftEchoPin, INPUT);
	pinMode(rightEchoPin, INPUT);
	pinMode(10, OUTPUT);
	pinMode(11, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);
	for (int i = 0; i < 7; i++)
	{
		pinMode(_GRAY_IO_PIN_[i], INPUT);
	}
	pinMode(grey_a, INPUT);
	pinMode(grey_b, INPUT);
	pinMode(grey_c, INPUT);
//	attachInterrupt(5, interrupt, FALLING);		//利用19号口作为中断
//初始化陀螺仪相关内容
	pinMode(2, INPUT);
	digitalWrite(2, LOW);
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	Serial.print("1");
	gyro.init();
	Serial.print("2");
	delay(3000);
}

void loop() {
	gyro.run();
	motor(gyro.getRelativeYaw()); 
	Serial.print("es:");
	Serial.println(gyro.getRelativeYaw());
	verify();
	barrier();
}
