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

int slide_time = 100;				//定义滑动时间
int wave_times = 2;
int wave_velocity=70;

//驱动
Motor motor(
	left_wheelPWM,
	left_wheel1,
	left_wheel2,
	right_wheelPWM,
	right_wheel1,
	right_wheel2
);

// 总流程
void run();

//PID循迹
PID pid(35, 0, 15);

//读取灰度传感器
void readsensor(void);
void read2sensor(void);
void read7sensor(void);
void check2sensor(void);
//循迹
void tracking_pid(void);

//黑线判断函数
int judge_line(void);
//超声波测距
float measure(void);



void setup() {
	Serial.begin(9600);
	pinMode(leftTrigPin, OUTPUT);
	pinMode(rightTrigPin, OUTPUT);
	pinMode(leftEchoPin, INPUT);
	pinMode(rightEchoPin, INPUT);
	pinMode(grey_a, INPUT);
	pinMode(grey_b, INPUT);
	motor.init();

#ifdef _LOW_START_FLAG_
{
	unsigned long _time = millis();
	motor.mot(200,200);
	while (millis() - _time < 500);
}
#endif
}

void loop()
{
	run();
}



void readsensor(void)
{
	read2sensor();
	read7sensor();
}

void read2sensor(void)
{
	Sa = digitalRead(grey_a);
	Sb = digitalRead(grey_b);
}
void check2sensor(void)
{
	Ca = digitalRead(grey_a);
	Cb = digitalRead(grey_b);
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
	
	//  //输出状态
	//  Serial.print("state: ");
	//  //Serial.println(S1*gray_id_1 | S2 * gray_id_2 | S3 * gray_id_3 | S4 * gray_id_4 | S5 * gray_id_5 | S6 * gray_id_6 | S7 * gray_id_7);
	//  Serial.print(S1);Serial.print(S2);Serial.print(S3);Serial.print(S4);Serial.print(S5);Serial.print(S6);Serial.print(S7);
	//  Serial.print("\noutput: ");
	//  Serial.println(output);

	//控制电机转速
	motor.mot((255 - output), (255 + output));
}


//黑线判断函数
int judge_line()
{
	read7sensor();
	if ((S1 == WHITE || S2 == WHITE) && (S3 == BLACK || S4 == BLACK || S5 == BLACK) && S6 == BLACK && S7==BLACK )
	{
		read7sensor();
		if ((S1 == WHITE || S2 == WHITE) && (S3 == BLACK || S4 == BLACK || S5 == BLACK)&& S6 == BLACK && S7 == BLACK)
		{
			read7sensor();
			if ((S1 == WHITE || S2 == WHITE) && (S3 == BLACK || S4 == BLACK || S5 == BLACK) && S6 == BLACK && S7 == BLACK)
			{
				read7sensor();
				if ((S1 == WHITE || S2 == WHITE) && (S3 == BLACK || S4 == BLACK || S5 == BLACK) && S6 == BLACK && S7 == BLACK) {
					Serial.println("stop");
					return 0;
				}
			}
		}
	}
	return 1;
}

//停止函数
void stop()
{
	int speed = 50;

	do{
		read2sensor();
		if (Sb == BLACK && speed > 0)
		{
			speed = -speed;
		}
		motor.mot(speed,speed);
	} while(Sa != BLACK);

	motor.mot(0,0);
}

//调整函数
void adjust()
{
	int times = 0;
	int velocity = wave_velocity;
	int flag;
	read2sensor();
	while (velocity > 30)
	{
		read2sensor();
		if (Sa == BLACK && Sb == WHITE)
			flag = 1;
		else if (Sa == WHITE && Sb == BLACK)
			flag = -1;
		else if (Sa == BLACK && Sb == BLACK)
			flag = 0;
		motor.mot(velocity*flag, velocity*flag);
		delay(10);
		times++;
		if (times == wave_times)
		{
			velocity--;
			times = 0;
		}
	}
	motor.mot(0, 0);
	delay(3000);
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

void run(void)
{
#ifdef _STOP_1_FLAG_
	unsigned long _stop_1_time = millis();
	while (millis() - _stop_1_time < _STOP_1_READY_TIME_)
		tracking_pid();
	
	while(1)
	{
		tracking_pid();
	}
	// while(judge_line())
	// {
	// 	tracking_pid();
	// }
	// while(1)
	// {
	// 	motor.mot(0,0);
	// }
	//stop();
	//adjust();
	//delay(3000);
#else
	tracking_pid();
#endif
}
