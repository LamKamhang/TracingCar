#include "Motor.h"
#include "PID.h"
#include "interface.h"

int S1, S2, S3, S4, S5, S6, S7, Sa, Sb;		//七路巡线返回值
int C1, C2, C3, C4, C5, C6, C7, Ca, Cb;	    //返回值确认

const unsigned char gray_id_1 = 1;
const unsigned char gray_id_2 = 1 << 1;
const unsigned char gray_id_3 = 1 << 2;
const unsigned char gray_id_4 = 1 << 3;
const unsigned char gray_id_5 = 1 << 4;
const unsigned char gray_id_6 = 1 << 5;
const unsigned char gray_id_7 = 1 << 6;

//驱动
Motor motor(
	left_wheelPWM,
	left_wheel1,
	left_wheel2,
	right_wheelPWM,
	right_wheel1,
	right_wheel2
);

//PID循迹
PID pid(40, 0.02, 30);

//读取七路灰度传感器
void readsensor(void);
void read7sensor(void);
//循迹
void tracking_pid(void);

//超声波测距
float measure(void);
//避障判断函数
void avoid(void);
//避障函数
void avoid_Barrier(void);


void setup() {
	Serial.begin(9600);
	pinMode(leftTrigPin, OUTPUT);
	pinMode(rightTrigPin, OUTPUT);
	pinMode(leftEchoPin, INPUT);
	pinMode(rightEchoPin, INPUT);
	pinMode(grey_a, INPUT);
	pinMode(grey_b, INPUT);
	motor.init();
}

void loop()
{
    tracking_pid();
}



void readsensor(void)
{
	S1 = digitalRead(sensor1);
	S2 = digitalRead(sensor2);
	S3 = digitalRead(sensor3);
	S4 = digitalRead(sensor4);
	S5 = digitalRead(sensor5);
	S6 = digitalRead(sensor6);
	S7 = digitalRead(sensor7);
	Sa = digitalRead(grey_a);
	Sb = digitalRead(grey_b);
}

void read7sensor(void)
{
	S1 = digitalRead(sensor1);
	S2 = digitalRead(sensor2);
	S3 = digitalRead(sensor3);
	S4 = digitalRead(sensor4);
	S5 = digitalRead(sensor5);
	S6 = digitalRead(sensor6);
	S7 = digitalRead(sensor7);
}

//循迹
void tracking_pid()
{
	// first readsensor
	read7sensor();
	int output = pid.get_output(S1*gray_id_1 | S2 * gray_id_2 | S3 * gray_id_3 | S4 * gray_id_4 | S5 * gray_id_5 | S6 * gray_id_6 | S7 * gray_id_7);
				
	// //输出状态
	// Serial.print("state: ");
	// Serial.println(S1*gray_id_1 | S2 * gray_id_2 | S3 * gray_id_3 | S4 * gray_id_4 | S5 * gray_id_5 | S6 * gray_id_6 | S7 * gray_id_7);
	// Serial.print("output: ");
	// Serial.println(output);

	//控制电机转速
	motor.mot(255 + output, 255 - output);
}

//超声波测距
float measure()
{
	//读取左超声波传感器
	digitalWrite(leftTrigPin, LOW);
	delay(2);
	digitalWrite(leftTrigPin, HIGH);
	delay(10);
	digitalWrite(leftTrigPin, LOW);
	float left_distance = pulseIn(leftEchoPin, HIGH) * 0.017;
	Serial.print("left diatance:");
	Serial.println(left_distance);
	//读取右超声波传感器
	digitalWrite(rightTrigPin, LOW);
	delay(2);
	digitalWrite(rightTrigPin, HIGH);
	delay(10);
	digitalWrite(rightTrigPin, LOW);
	float right_distance = pulseIn(rightEchoPin, HIGH) * 0.017;
	Serial.print("right diatance:");
	Serial.println(right_distance);
	//获得最短距离
	float distance = left_distance < right_distance ? left_distance : right_distance;
	Serial.print("diatance:");
	Serial.println(distance);
	return distance;
}

//避障判断函数
void avoid()
{
	time = millis();
	motor.mot(255, 255);
	float distance;
	while (1)
	{
		tracking_pid();
		if (millis() - time > reach_barrier_time)
		{
			distance = measure();
			if (distance < 25 && distance>5) {
				delay(delay_time);
				distance = measure();
				if (distance < 25 && distance>5) {
					delay(delay_time);
					distance = measure();
					if (distance < 25 && distance>5) 
						avoid_Barrier();
				}
			}
		}
	}
}

//避障函数
void avoid_Barrier()
{
	motor.changeState(_LEFT_ROTATION_);
	delay(600);
	motor.changeState(_FAST_STRAIGHT_);
	delay(500);
	motor.changeState(_RIGHT_ROTATION_);
	delay(600);
	motor.changeState(_FAST_STRAIGHT_);
	while (1)
	{
		readsensor();
		if (S1 == WHITE && S3 == WHITE && S5 == WHITE && S7 == WHITE && Sa == BLACK)
		{
			readsensor();
			if (S1 == WHITE && S3 == WHITE && S5 == WHITE && S7 == WHITE && Sa == BLACK)
			{
				readsensor();
				if (S1 == WHITE && S3 == WHITE && S5 == WHITE && S7 == WHITE && Sa == BLACK)
				{
					motor.changeState(_LEFT_ROTATION_);
					delay(600);
				}
			}
		}
	}
}