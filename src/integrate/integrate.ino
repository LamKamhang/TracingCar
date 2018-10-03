
#include "Motor.h"
#include "PID.h"
#include "interface.h"

int S1, S2, S3, S4, S5, S6, S7, Sa, Sb, Sc;		//七路巡线返回值
int C1, C2, C3, C4, C5, C6, C7, Ca, Cb, Cc;	    //返回值确认

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
const int delay_time = 3;		//定义延时时间

//PID循迹
PID pid(40, 0.001, 20);

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


//读取灰度传感器
void readsensor(void);
void read3sensor(void);
void read7sensor(void);
void check3sensor(void);
//循迹
void low_start();
void tracking_pid(void);

//黑线判断函数
int judge_line(void);
void adjust(void);
void catch_adjust();
void place_adjust();
//超声波测距
float measure(void);
//避障判断函数
void avoid();
//避障函数
void avoid_Barrier();



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
	low_start();
#endif
}

void loop(){
//   read7sensor();
//   Serial.print(S1);
// 	Serial.print(S2);
//  Serial.print(S3);
//  Serial.print(S4);
//  Serial.print(S5);
//  Serial.print(S6);
//  Serial.print(S7);
//  Serial.println();
	run();
//	avoid_Barrier();
}



void readsensor(void)
{
	read3sensor();
	read7sensor();
}

void read3sensor(void)
{
	Sa = digitalRead(grey_a);
	Sb = digitalRead(grey_b);
	Sc = digitalRead(grey_c);
}
void check3sensor(void)
{
	Ca = digitalRead(grey_a);
	Cb = digitalRead(grey_b);
	Cc = digitalRead(grey_c);
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

void low_start()
{
	unsigned long _time = millis();
	motor.mot(200,200);
	while (millis() - _time < 500);
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
	boolean flag = 1;
	read3sensor();
	if ((Sa + Sb) && Sc == WHITE)// a 或者 b 是黑
	{
		delay(3);
		read3sensor();
		if ((Sa + Sb) && Sc == WHITE)
		{
			delay(3);
			read3sensor();
			if ((Sa + Sb) && Sc == WHITE){
				delay(3);
				read3sensor();
				if ((Sa + Sb) && Sc == WHITE)
					flag = 0;
			}
		}
	}
	return flag;
}


//调整函数
void adjust()
{
	int times = 0;
	int times2 = 0;
	int velocity = wave_velocity;
	int flag;
	unsigned long _adjust_time = millis();
	while (millis() - _adjust_time < _ADJUST_TIME)
	{
		read3sensor();
		if (Sa == BLACK && Sb == BLACK)
		{
			flag = 0;
			times2++;
			if (times2 > 4)
				break;
		}
		else if (Sa == BLACK && Sb == WHITE)
			flag = 1;
		else
			flag = -1;
		motor.mot(velocity*flag, velocity*flag);
		delay(10);
		times++;
		if (times == wave_times)
		{
			if (velocity > 30)
			{
				velocity-=10;
				times = 0;
			}
		}
	}
	motor.mot(0, 0);
}

//夹取后退
void catch_adjust()
{
	motor.mot(-_CATCH_SPEED, -_CATCH_SPEED);
	delay(_CATCH_BACK_TIME);
	motor.mot(10, 10);
	delay(10);
	motor.mot(0, 0);
	delay(2000);
	for (int i = 0; i < 4; ++i)
	{
		motor.mot(_CATCH_SPEED, _CATCH_SPEED);
		delay(_CATCH_FORWARD_TIME);
		motor.mot(-10, -10);
		delay(10);
		motor.mot(0, 0);
		delay(2000);
	}
}

// 放置后退
void place_adjust()
{
	motor.mot(-_CATCH_SPEED, -_CATCH_SPEED);
	delay(_PLACE_BACK_TIME);
	motor.mot(10, 10);
	delay(10);
	motor.mot(0, 0);
	delay(2000);
	for (int i = 0; i < 4; ++i)
	{
		motor.mot(_CATCH_SPEED, _CATCH_SPEED);
		delay(_PLACE_FORWARD_TIME);
		motor.mot(-10, -10);
		delay(10);
		motor.mot(0, 0);
		delay(2000);
	}
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
	float distance = measure();
	while(1)
	{
		tracking_pid();
		distance = measure();
		if (distance < 25 && distance>5) {
			delay(delay_time);
			distance = measure();
			if (distance < 25 && distance>5) {
				delay(delay_time);
				distance = measure();
				if (distance < 25 && distance>5) 
					avoid_Barrier();
					break;
			}
		}
		else if (distance < 5)
		{
			motor.mot(-200, -200);
			delay(500);
		}
	}
}

//避障函数
void avoid_Barrier()
{
	motor.mot(-100,200);
	delay(600);
	motor.mot(200,200);
	delay(1000);
	motor.mot(200,-100);
	delay(1200);
  	motor.mot(200,200);
	// while (1)
	// {
	// 	readsensor();
	// 	if (S1 == WHITE && S3 == WHITE && S5 == WHITE && S7 == WHITE && Sa == BLACK)
	// 	{
	// 		readsensor();
	// 		if (S1 == WHITE && S3 == WHITE && S5 == WHITE && S7 == WHITE && Sa == BLACK)
	// 		{
	// 			readsensor();
	// 			if (S1 == WHITE && S3 == WHITE && S5 == WHITE && S7 == WHITE && Sa == BLACK)
	// 			{
	// 				motor.changeState(_LEFT_ROTATION_);
	// 				delay(500);
	// 				break;
	// 			}
	// 		}
	// 	}
	// }
}

void run(void)
{
#ifdef _STOP_1_FLAG_
	unsigned long _stop_1_time = millis();
	while (millis() - _stop_1_time < _STOP_1_READY_TIME_)
		tracking_pid();
	
	while(judge_line())
	{
		tracking_pid();
	}
	adjust();
	delay(3000);	
	catch_adjust();
#endif
	low_start();
	unsigned long _some_time = millis();
	while (millis() - _some_time < _BARRIER_TIME)
		tracking_pid();

	avoid();

#ifdef _STOP_2_FLAG_
{
	unsigned long _stop_2_time = millis();
	while (millis() - _stop_2_time < _STOP_2_READY_TIME_)
		tracking_pid();

	while(judge_line())
	{
		tracking_pid();
	}
	adjust();
	delay(3000);	
	place_adjust();
}
#endif 
	low_start();
	unsigned long _final_time = millis();
	while(millis() - _final_time < 3000)
		tracking_pid();
    
	do{
		tracking_pid();
		read7sensor();
	}while(S1+S2+S3+S4+S5+S6+S7);

	motor.mot(200,200);
	delay(500);
	motor.mot(0,0);

  	delay(3000);
}
