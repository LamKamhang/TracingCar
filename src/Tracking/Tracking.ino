#include "Motor.h"
#include "PID.h"
#include "util.hpp"
#include "MyServo.h"

//定义七路灰度传感器接口
#define sensor1 30   
#define sensor2 32
#define sensor3 34
#define sensor4 36
#define sensor5 38
#define sensor6 40
#define sensor7 42

//定义黑白
#define BLACK 0
#define WHITE 1

//定义车轮驱动接口
const int left_wheel1 = 9;
const int left_wheel2 = 10;
const int left_wheelPWM = 8;
const int right_wheel1 = 5;
const int right_wheel2 = 6;
const int right_wheelPWM = 4;

//定义超声波探测引脚
const int leftTrigPin = 23;
const int leftEchoPin = 22;
const int rightTrigPin = 44;
const int rightEchoPin = 45;

//定义三路灰度传感器
const int grey_a = 18;		//前方传感器
const int grey_b = 19;		//中央传感器
const int grey_c = 20;		//后方传感器

int S1, S2, S3, S4, S5, S6, S7, Sa, Sb;		//七路巡线返回值
int C1, C2, C3, C4, C5, C6, C7, Ca, Cb;	 //返回值确认
////////////////////////////////////////////////////////////
int servo_state[6];
int reset_state[6] = { 86,110,70,155,14,32 };

int seat_state;
int origin_seat_state = 74;

int time;		//定义时间变量

int wave_times = 2;
int wave_velocity=70;

int reach_line_time1 = 4000;		//定义到达第一根线的时间
int reach_line_time2 = 2000;		//定义到达第二根线的时间
int arrive_time = 2000;				//定义到达终点的时间
int reach_barrier_time = 2000;		//定义到达障碍区的时间
int slide_time = 100;				//定义滑动时间
int back_time = 500;			//定义倒退时间
int forward_time = 300;			//定义前进时间
const int delay_time = 3;		//定义延时时间

int stop_state;		//定义停止变量
int end_of_race;	//判定比赛是否结束
//////////////////////////////////////////////////////////
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

//机械臂
MyServo servo(Serial3);

//读取七路灰度传感器
void readsensor()
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

void checksensor()
{
	C1 = digitalRead(sensor1);
	C2 = digitalRead(sensor2);
	C3 = digitalRead(sensor3);
	C4 = digitalRead(sensor4);
	C5 = digitalRead(sensor5);
	C6 = digitalRead(sensor6);
	C7 = digitalRead(sensor7);
}

//循迹
void tracking_pid()
{
	// first readsensor
	readsensor();
	int output = pid.get_output(S1*gray_id_1 | S2 * gray_id_2 | S3 * gray_id_3 | S4 * gray_id_4 | S5 * gray_id_5 | S6 * gray_id_6 | S7 * gray_id_7);
				
	//输出状态
	Serial.print("state: ");
	Serial.println(S1*gray_id_1 | S2 * gray_id_2 | S3 * gray_id_3 | S4 * gray_id_4 | S5 * gray_id_5 | S6 * gray_id_6 | S7 * gray_id_7);
	Serial.print("output: ");
	Serial.println(output);

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

//黑线判断函数
void judge_line()
{
	readsensor();
	if ((S1 == WHITE || S2 == WHITE) && (S3 == BLACK || S4 == BLACK || S5 == BLACK) && S6 == BLACK && S7==BLACK )
	{
		readsensor();
		if ((S1 == WHITE || S2 == WHITE) && (S3 == BLACK || S4 == BLACK || S5 == BLACK)&& S6 == BLACK && S7 == BLACK)
		{
			readsensor();
			if ((S1 == WHITE || S2 == WHITE) && (S3 == BLACK || S4 == BLACK || S5 == BLACK) && S6 == BLACK && S7 == BLACK)
			{
				readsensor();
				if ((S1 == WHITE || S2 == WHITE) && (S3 == BLACK || S4 == BLACK || S5 == BLACK) && S6 == BLACK && S7 == BLACK) {
					stop_state = 1;
				}
			}
		}
	}
}

//停止函数
void stop()
{
	motor.mot(50, 50);
	if (Sa == BLACK) {
		readsensor();
		if (Sa == BLACK) {
			delay(slide_time);
			motor.mot(-100, -100);
			delay(120);
			motor.mot(0, 0);
		}
	}
}

//调整函数
void adjust()
{
	int times = 0;
	int velocity = wave_velocity;
	int flag;
	readsensor();
	while (velocity > 30)
	{
		readsensor();
		if (Sa == BLACK && Sb == WHITE)
			flag = 1;
		else if (Sa == WHITE && Sb == BLACK)
			flag = -1;
		else if (Sa == WHITE && Sb == WHITE)
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

//终点判断函数
void arrive()
{
	readsensor();
	if (S1 == WHITE && S2 == WHITE && S3 == WHITE && S4 == WHITE && S5 == WHITE && S6 == WHITE && S7 == WHITE)
	{
		delay(delay_time);
		readsensor();
		if (S1 == WHITE && S2 == WHITE && S3 == WHITE && S4 == WHITE && S5 == WHITE && S6 == WHITE && S7 == WHITE)
		{
			delay(delay_time);
			readsensor();
			if (S1 == WHITE && S2 == WHITE && S3 == WHITE && S4 == WHITE && S5 == WHITE && S6 == WHITE && S7 == WHITE)
				end_of_race = 1;
		}
	}
}

//舵机状态改变函数
void define_servo(int D1, int D2, int D3, int D4, int D5, int D6)
{
	int d[6];
	int t[6];
	int d_max = 0;
	int t_max = 0;
	int speed;
	int place[6] = { D1, D2, D3, D4, D5, D6 };
	for (int i = 0; i < 6; i++) {
		d[i] = abs(place[i] - servo_state[i]);
		if (d[i] > d_max) d_max = d[i];
		servo_state[i] = place[i];
	}
	for (int i = 0; i < 6; i++) {
		if (d_max >= 60) speed = d[i] * 16 / d_max;
		else speed = d[i] * 10 / d_max;
		if ((d[i] > 140) && (i == 1)) speed = 20;
		if (speed > 20) speed = 20;
		if (speed <= 5) speed = 5;
		servo.moveServo(i, place[i], speed);
		t[i] = d[i] * 112 / speed;
		if (t[i] > t_max) t_max = t[i];
	}
	delay(t_max);
}

//舵机初始化函数
void reset_servo()
{
	define_servo(reset_state[0], reset_state[1], reset_state[2], reset_state[3], reset_state[4], reset_state[5]);
}

//倒退函数
void step_back()
{
	motor.mot(-100, -100);
	delay(back_time);
	motor.mot(0, 0);
}

//前进函数
void step_forward()
{
	motor.mot(100, 100);
	delay(forward_time);
	motor.mot(0, 0);
}

//夹取动作主函数
void begin_catch()
{
	step_back();
	for (int i = 0; i < 0; i++)
	{
		catch_block();
		step_forward();
	}
	catch_block();
	motor.mot(255,255);
}

//夹取动作子函数
void catch_block()
{
	define_servo(32, 110, 70, 155, 14, 32);		//转向
	define_servo(32, 56, 86, 124, 14, 32);		//到达夹取位置
	define_servo(32, 56, 86, 124, 65, 32);		//夹取
	define_servo(32, 110, 70, 155, 65, 32);		//回到转向位置
	define_servo(86, 110, 70, 155, 65, 32);		//回到上方
	define_servo(86, 110, 70, 172, 65, 32);		//放下动作
	define_servo(86, 110, 70, 172, 14, 32);		//松开
	reset_servo();		//复位
}

void setup() {
	end_of_race = 0;
	Serial.begin(9600);
	pinMode(leftTrigPin, OUTPUT);
	pinMode(rightTrigPin, OUTPUT);
	pinMode(leftEchoPin, INPUT);
	pinMode(rightEchoPin, INPUT);
	pinMode(grey_a, INPUT);
	pinMode(grey_b, INPUT);
	motor.init();
	for (int i = 0; i < 6; i++)
		servo_state[i] = reset_state[i];
	//reset_servo();
}

//主函数
void begin()
{
	/*由起点至上货点*/
	motor.mot(200, 200);
	delay(500);
	time = millis();
	while (1)
	{
		tracking_pid();
		if (millis() - time > reach_line_time1)
			judge_line();
		if (stop_state)
		{
			break;
		}
	}
	stop();
	adjust();
	
	/*上货动作*/
	delay(3000);

	/*由上货点通过避障区*/
	avoid();

	/*由避障区至落货区*/
	time = millis();
	stop_state = 0;
	while (1)
	{
		tracking_pid();
		if (millis() - time > reach_line_time1)
			judge_line();
		if (stop_state)
		{
			break;
		}
	}
	stop();
	adjust();

	/*卸货动作*/
	delay(3000);

	/*由落货区到终点*/
	time = millis();
	while (1)
	{
		tracking_pid();
		if (millis() - time > arrive_time)
			arrive();
		if (end_of_race)
		{
			delay(1000);
			motor.mot(-180, -180);
			delay(80);
			motor.mot(0, 0);
			while (1)
				{
				}
			break;
		}
	}
}

void loop() {
	//begin();
  readsensor();      
  //输出状态
  Serial.print("state: ");
  Serial.println(S1*gray_id_1 | S2 * gray_id_2 | S3 * gray_id_3 | S4 * gray_id_4 | S5 * gray_id_5 | S6 * gray_id_6 | S7 * gray_id_7);
  delay(1000);
}
