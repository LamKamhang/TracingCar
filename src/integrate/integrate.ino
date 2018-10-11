
#include "Motor.h"
#include "PID.h"
#include "interface.h"
#include "MyServo.h"
#include "scan.h"

int S1, S2, S3, S4, S5, S6, S7, Sa, Sb, Sc, Sl, Sr;		//七路巡线返回值

const unsigned char gray_id_1 = 1;
const unsigned char gray_id_2 = 1 << 1;
const unsigned char gray_id_3 = 1 << 2;
const unsigned char gray_id_4 = 1 << 3;
const unsigned char gray_id_5 = 1 << 4;
const unsigned char gray_id_6 = 1 << 5;
const unsigned char gray_id_7 = 1 << 6;

//转盘位置
int plate_state;
int reset_state=15;
int target_state[6] = { 36,54,70,88,106,125 };

//夹取和放置次数
int count1 = 0;
int count2 = 0;
int end_of_catch=0;
int end_of_place=0;

//机械臂位置
int servo_state[5];
int first_catch_state[5] = { 130,90,68,80,27 };
int second_catch_state[5] = { 130,57,117,73,27 };
int first_place_state[5] = { 125,106,72,97,27 };
int second_place_state[5] = { 125,117,47,93,27 };

//PID循迹
PID pid(40, 0.001, 20);

//机械臂
MyServo servo(Serial3);

//扫码模块
ScanModule scanM;

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
void read2sensor(void);
void read3sensor(void);
void read7sensor(void);
//循迹
void low_start();
void tracking_pid(void);
//黑线判断函数
int judge_line(void);
//机械臂函数
void define_servo(int D1, int D2, int D3, int D4, int D5);
void first_catch_servo();
void second_catch_servo();
void first_place_servo();
void second_place_servo();
//转盘函数
void reset_plate();
void define_plate(char);
//调整函数
void adjust(void);
void slight_adjust();
void avoid_adjust();
void catch_adjust();
void place_adjust();
int check_state(int);
int cal_state(int);
//夹取函数
void catch_first();
void catch_second();
//放置函数
void place_first();
void place_second();
//超声波测距
float measure(void);
//避障函数
void avoid();
void avoid_Barrier();
//重复扫描函数
char scans();


void setup() {
	Serial.begin(9600);
	pinMode(leftTrigPin, OUTPUT);
	pinMode(rightTrigPin, OUTPUT);
	pinMode(leftEchoPin, INPUT);
	pinMode(rightEchoPin, INPUT);
	pinMode(grey_a, INPUT);
	pinMode(grey_b, INPUT);
	pinMode(grey_c, INPUT);
	pinMode(grey_l, INPUT);
	pinMode(grey_r, INPUT);
	motor.init();
	scanM.init();
	first_catch_servo();
	for (int i = 0; i < 6; i++)
	{
		servo_state[i] = first_catch_state[i];
	}
	reset_plate();
#ifdef _LOW_START_FLAG_
	low_start();
#endif
}

void loop(){
	//read7sensor();
	//Serial.print(S1);
	//Serial.print(S2);
	//Serial.print(S3);
	//Serial.print(S4);
	//Serial.print(S5);
	//Serial.print(S6);
	//Serial.print(S7);
	//Serial.println();
	//read3sensor();
	//Serial.print(Sa);
	//Serial.print(Sb);
	//Serial.print(Sc);
	//Serial.println();
	//read2sensor();
	//Serial.print(Sl);
	//Serial.print(Sr);
	//Serial.println();
	//delay(1000);
//	tracking_pid();
//	run();
//	measure();
	//reset_plate();
	//delay(1000);
	//define_plate(5);
	//while (1) {}
//	avoid();
//	tracking_pid();
	Serial.println(scanM.scan() + 0);
//  while (1);
//	avoid_adjust();
//	slight_adjust();
//	place_first();

}

//读取灰度传感器
void read2sensor(void)
{
	Sr = digitalRead(grey_l);
	Sl = digitalRead(grey_r);
}
void read3sensor(void)
{
	Sa = digitalRead(grey_a);
	Sb = digitalRead(grey_b);
	Sc = digitalRead(grey_c);
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
void low_start()
{
	unsigned long _time = millis();
	motor.mot(200,200);
	while (millis() - _time < 500);
}
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
			if ((Sa + Sb) && Sc == WHITE) {
				delay(3);
				read3sensor();
				if ((Sa + Sb) && Sc == WHITE)
				{
					flag = 0;
					motor.mot(70, 70);
				}
			}
		}
	}
	return flag;
}

//机械臂函数
void define_servo(int D1, int D2, int D3, int D4, int D5)
{
	int d[5];
	int t[5];
	int d_max = 0;
	int t_max = 0;
	int speed;
	int place[5] = { D1, D2, D3, D4, D5 };
	for (int i = 0; i < 5; i++) {
		d[i] = abs(place[i] - servo_state[i]);
		if (d[i] > d_max) d_max = d[i];
		servo_state[i] = place[i];
	}
	for (int i = 0; i < 5; i++) {
		if (d_max >= 60) speed = d[i] * 16 / d_max;
		else speed = d[i] * 10 / d_max;
		if ((d[i] > 140) && (i == 1)) speed = 20;
		if (speed > 20) speed = 20;
		if (speed <= 5) speed = 5;
		servo.moveServo(i, place[i], speed);
		t[i] = d[i] * 112 / speed;
		if (t[i] > t_max) t_max = t[i];
	}
	delay(t_max+1000);
}
void first_catch_servo()
{
	define_servo(first_catch_state[0], first_catch_state[1], first_catch_state[2],
		first_catch_state[3], first_catch_state[4]);
}
void second_catch_servo()
{
	define_servo(second_catch_state[0], second_catch_state[1], second_catch_state[2],
		second_catch_state[3], second_catch_state[4]);
}
void first_place_servo()
{
	define_servo(first_place_state[0], first_place_state[1], first_place_state[2],
		first_place_state[3], first_place_state[4]);
}
void second_place_servo()
{
	define_servo(second_place_state[0], second_place_state[1], second_place_state[2],
		second_place_state[3], second_place_state[4]);
}

//转盘函数
void reset_plate()
{
	int d;
	int speed;
	d = abs(reset_state - plate_state);
	plate_state = reset_state;
	if (d >= 60) speed =  16 ;
	else speed = 10;
	servo.moveServo(5, reset_state, speed);
}
void define_plate(char order)
{
	int d;
	int t;
	int speed=10;
	for (int i = 0; i < order; i++)
	{
		d = abs(target_state[i] - plate_state);
		servo.moveServo(5, target_state[i], speed);
		t = d * 112 / speed;
		delay(t);
	}
	plate_state = target_state[order - 1];
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
	times = 0;
	times2 = 0;
	_adjust_time = millis();
	velocity = wave_velocity;
	while (millis() - _adjust_time < _ADJUST_TIME2)
	{
		read2sensor();
		read7sensor();
		cal_state(Sr*gray_id_1 | Sl * gray_id_2 | S5 * gray_id_3 | S3 * gray_id_4);
		read3sensor();
		if (Sa == BLACK && Sb == BLACK)
		{
			flag = 0;
		}
		else if (Sa == BLACK && Sb == WHITE)
			flag = 1;
		else
			flag = -1;
		motor.mot(velocity*flag, velocity*flag);
		delay(10);
		read3sensor();
		if (Sa == BLACK && Sb == BLACK && check_state(1) == 0 && check_state(2) == 0)
		{
			times2++;
			if (times2 > 4)
				break;
		}
		times++;
		if (times == wave_times)
		{
			if (velocity > 30)
			{
				velocity -= 10;
				times = 0;
			}
		}
	}
	motor.mot(0, 0);
}
void slight_adjust() 
{
	int times = 0;
	int times2 = 0;
	int velocity = wave_velocity;
	int flag;
	unsigned long _adjust_time = millis();
	while (millis() - _adjust_time < _ADJUST_TIME)
	{
		//flag = check_state(1);
		//motor.mot(-velocity * flag, velocity * flag);
		//delay(100);
		read2sensor();
		read7sensor();
		cal_state(Sr*gray_id_1 | Sl * gray_id_2 | S5 * gray_id_3 | S3 * gray_id_4);
		if (check_state(1) == 0 && check_state(2)==0 )
		{
			times2++;
			if (times2 > 4)
				break;
		}
		times++;
		if (times == wave_times)
		{
			if (velocity > 30)
			{
				velocity -= 10;
				times = 0;
			}
		}
	}
	//times = 0;
	//times2 = 0;
	//velocity = wave_velocity;
	//_adjust_time = millis();
	//while (millis() - _adjust_time < _ADJUST_TIME)
	//{
	//	flag = check_state(2);
	//	motor.mot(-velocity * flag, velocity * flag);
	//	delay(100);
	//	if ( check_state(2) == 0)
	//	{
	//		times2++;
	//		if (times2 > 4)
	//			break;
	//	}
	//	times++;
	//	if (times == wave_times)
	//	{
	//		if (velocity > 30)
	//		{
	//			velocity -= 10;
	//			times = 0;
	//		}
	//	}
	//}
	motor.mot(0, 0);
}
void avoid_adjust()
{
	int times = 0;
	int times2 = 0;
	int velocity = wave_velocity;
	int flag;
	unsigned long _adjust_time = millis();
	while (millis() - _adjust_time < _ADJUST_TIME)
	{
		flag = check_state(1);
		motor.mot(-velocity * flag, velocity * flag);
		delay(100);
		if (check_state(1) == 0)
		{
			times2++;
			if (times2 > 4)
				break;
		}
		times++;
		if (times == wave_times)
		{
			if (velocity > 30)
			{
				velocity -= 10;
				times = 0;
			}
		}
	}
	motor.mot(0, 0);
}
int check_state(int i)
{
	read7sensor();
	read2sensor();
	int flag1;
	int flag2;

	if (S3 == BLACK && S5 == BLACK)
		flag1 = 0;
	else if (S3 == WHITE && S5 == BLACK)
		flag1 = -1;
	else if (S3 == BLACK && S5 == WHITE)
		flag1 = 1;
	else
		flag1 = 2;

	if (Sl == BLACK && Sr == BLACK)
		flag2 = 0;
	else if (Sl == WHITE && Sr == BLACK)
		flag2 = 1;
	else if (Sl == BLACK && Sr == WHITE)
		flag2 = -1;
	else
		flag2 = 0;

	if (i == 1)
		return flag1;
	else
		return flag2;
}
int cal_state(int state)
{
	int output;
	switch (state)
	{
	case G1111:
	case G1100:
		motor.mot(0, 0);
		break;
	case G0111:
	case G0100:
	case G1110:
		motor.mot(50, -50);
		delay(100);
		break;
	case G1011:
	case G1000:
	case G1101:
		motor.mot(-50, 50);
		delay(100);
		break;
	case G1001:
		motor.mot(-70, 70);
		delay(100);
		break;
	case G0110:
		motor.mot(70, -70);
		delay(100);
		break;
	case G1010:
		motor.mot(-50, 70);
		delay(100);
		break;
	case G0101:
		motor.mot(70, -50);
		delay(100);
		break;
	}
}

void place_adjust()
{
	motor.mot(-_CATCH_SPEED, -_CATCH_SPEED);
	delay(_PLACE_BACK_TIME);
	motor.mot(10, 10);
	delay(10);
	motor.mot(0, 0);
	slight_adjust();
	place_first();
	place_second();
	for (int i = 0; i < 2; ++i)
	{
		motor.mot(_CATCH_SPEED, _CATCH_SPEED);
		delay(_PLACE_FORWARD_TIME);
		motor.mot(-10, -10);
		delay(10);
		motor.mot(0, 0);
		slight_adjust();
		place_second();
		if (end_of_place == 1)
			break;
		place_first();
		if (end_of_place == 1)
			break;

		motor.mot(_CATCH_SPEED, _CATCH_SPEED);
		delay(_PLACE_FORWARD_TIME);
		motor.mot(-10, -10);
		delay(10);
		motor.mot(0, 0);
		slight_adjust();
		place_first();
		if (end_of_place == 1)
			break;
		place_second();
		if (end_of_place == 1)
			break;
	}
}
void catch_adjust()
{
	motor.mot(-_CATCH_SPEED, -_CATCH_SPEED);
	delay(_CATCH_BACK_TIME);
	motor.mot(10, 10);
	delay(10);
	motor.mot(0, 0);
	slight_adjust();
	catch_first();
	catch_second();
	for (int i = 0; i < 2; ++i)
	{
		motor.mot(_CATCH_SPEED, _CATCH_SPEED);
		delay(_CATCH_FORWARD_TIME);
		motor.mot(-10, -10);
		delay(10);
		motor.mot(0, 0);
		slight_adjust();
		catch_second();
		if (end_of_catch == 1)
			break;
		catch_first();
		if (end_of_catch == 1)
			break;

		motor.mot(_CATCH_SPEED, _CATCH_SPEED);
		delay(_CATCH_FORWARD_TIME);
		motor.mot(-10, -10);
		delay(10);
		motor.mot(0, 0);
		slight_adjust();
		catch_first();
		if (end_of_catch == 1)
			break;
		catch_second();
		if (end_of_catch == 1)
			break;
	}
}

//夹取函数
void catch_first()
{
	char res;
	first_catch_servo();
	res = scans();
	if (res != 0)
	{
		define_plate(res);
		servo.runActionGroup(2);
		delay(6000);
		reset_plate();
		count1++;
		if (count1 == _BLOCK_NUMBER_)
			end_of_catch = 1;
	}
}
void catch_second()
{
	char res;
	second_catch_servo();
	res = scans();
	if (res != 0 && count1 != _BLOCK_NUMBER_)
	{
		define_plate(res);
		servo.runActionGroup(3);
		delay(6000);
		reset_plate();
		count1++;
		if (count1 == _BLOCK_NUMBER_)
			end_of_catch = 1;
	}
}

//放置函数
void place_first()
{
	char res;
	first_place_servo();
	res = scans();
	if (res != 0 && count2 != _BLOCK_NUMBER_)
	{
		define_plate(res);
		servo.runActionGroup(1);
		delay(1100);
		reset_plate();
		count2++;
		if (count2 == _BLOCK_NUMBER_)
			end_of_place = 1;
	}
}
void place_second()
{
	char res;
	second_place_servo();
	res = scans();
	if (res != 0 && count2 != _BLOCK_NUMBER_)
	{
		define_plate(res);
		servo.runActionGroup(1);
		delay(1100);
		reset_plate();
		count2++;
		if (count2 == _BLOCK_NUMBER_)
			end_of_place = 1;
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
	if (distance < 25 && distance>=0)
	{
		Serial.print("diatance:");
		Serial.println(distance);
	}
	return distance;
}

//避障函数
void avoid()
{
	float distance;
	unsigned long time;
	time = millis();
	distance = measure();
	while(1)
	{
		tracking_pid();
		if (millis() - time > _MESSURE_TIME)
		{
			distance = measure();
			time = millis();
		}
		if (distance < 20 && distance>0) {
			avoid_Barrier();
			break;
		}
	}
}
void avoid_Barrier()
{
	motor.mot(-100, -100);
	delay(300);
	motor.mot(-100,200);
	delay(_ROTATION_TIME);
	motor.mot(200,200);
	delay(700);
	motor.mot(200,-100);
	delay(_ROTATION_TIME);
  	motor.mot(200,200);
	delay(800);
	motor.mot(200, -100);
	delay(_ROTATION_TIME-250);
	motor.mot(200, 200);
	while (1)
	 {
		int flag = 0;
	 	read7sensor();
		read3sensor();
	 	if ((S1+S2+S3+S4+S5+S6+S7)==0 && Sa == BLACK)
	 	{
	 		read7sensor();
	 		if ((S1 + S2 + S3 + S4 + S5 + S6 + S7) == 0 && Sa == BLACK)
	 		{
	 			read7sensor();
	 			if ((S1 + S2 + S3 + S4 + S5 + S6 + S7) == 0 && Sa == BLACK)
	 			{
					motor.mot(-100, -100);
					delay(50);
					motor.mot(-200,200);
					break;
	 			}
	 		}
	 	}
	 }
	while (1)
	{
		read7sensor();
		if ((S1 + S2 + S3 + S4 ) == 1)
			break;
	}
}

//重复扫描函数
char scans()
{
	char res;
	int order[] = { 1,1,1,-4,-1,-1 };
	for (int i = 0; i < 6; i++)
	{
		if (res = scanM.scan())
			break;
		else
			define_servo(servo_state[0], servo_state[1], servo_state[2],
				servo_state[3] + order[i], servo_state[4]);
	}
	return res;
}

//主函数
void run(void)
{
#ifdef _STOP_1_FLAG_
	unsigned long _stop_1_time = millis();
	while (millis() - _stop_1_time < _STOP_1_READY_TIME_)
		tracking_pid();

	while (judge_line())
	{
		tracking_pid();
	}
	adjust();
	catch_adjust();
	first_place_servo();
	pid.reset();
#endif
	low_start();
	unsigned long _some_time = millis();
	while (millis() - _some_time < _BARRIER_TIME)
		tracking_pid();
	motor.mot(50, 50);
	delay(100);
	avoid();
	pid.reset();

#ifdef _STOP_2_FLAG_
	{
		unsigned long _stop_2_time = millis();
		while (millis() - _stop_2_time < _STOP_2_READY_TIME_)
			tracking_pid();

		while (judge_line())
		{
			tracking_pid();
		}
		adjust();
		place_adjust();
	}
#endif 
	low_start();
	unsigned long _final_time = millis();
	while (millis() - _final_time < 3000)
		tracking_pid();

	do {
		tracking_pid();
		read7sensor();
	} while (S1 + S2 + S3 + S4 + S5 + S6 + S7);

	motor.mot(200, 200);
	delay(500);
	motor.mot(0, 0);

	while (1) {}
}