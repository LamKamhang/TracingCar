#include "Motor.h"
#include "PID.h"
#include "util.hpp"

//定义七路灰度传感器接口
#define sensor1 30   
#define sensor2 32
#define sensor3 34
#define sensor4 36
#define sensor5 38
#define sensor6 40
#define sensor7 42

//定义车轮驱动接口
const int left_front_wheel1 = 4;
const int left_front_wheel2 = 5;
const int right_front_wheel1 = 10;
const int right_front_wheel2 = 11;
const int left_behind_wheel1 = 9;
const int left_behind_wheel2 = 8;
const int right_behind_wheel1 = 7;
const int right_behind_wheel2 = 6;

//定义超声波探测引脚
const int leftTrigPin = 23;
const int leftEchoPin = 22;
const int rightTrigPin = 44;
const int rightEchoPin = 45;

//定义三路灰度传感器
const int grey_a = 19;		//左前的传感器，作为4号中断，进行减速，应调节current_velocity
const int grey_b = 18;		//中间靠前的传感器，作为5号中断，将速度减为0

const int BLACK = 1;
const int WHITE = 0;		//定义颜色

int S1, S2, S3, S4, S5, S6, S7;  //七路巡线返回值
const int t = 1;

int near_Barrier = 0;      //定义避障状态变量

float distance;				//定义最小距离变量
int stop_five_sec = 0;		//定义静止函数
int line = 6;
int flag = 0;        //定义状态变量
int next_line = 3;      
int next_flag = 0;

const unsigned char gray_id_1 = 1;
const unsigned char gray_id_2 = 1 << 1;
const unsigned char gray_id_3 = 1 << 2;
const unsigned char gray_id_4 = 1 << 3;
const unsigned char gray_id_5 = 1 << 4;
const unsigned char gray_id_6 = 1 << 5;
const unsigned char gray_id_7 = 1 << 6;

//循迹
Motor motor(
	left_front_wheel1,
	left_front_wheel2,
	right_front_wheel1,
	right_front_wheel2,
	left_behind_wheel1,
	left_behind_wheel2,
	right_behind_wheel1,
	right_behind_wheel2
);

/**************************************************************/
// PID循迹
    // kp,  ki,   kd
PID pid(10, 0.02, 20);

void tracking_pid()
{
  // first readsensor
  readsensor();
  // output --> negative --> need to turn left
  // output --> positive --> need to turn right
  int output = pid.get_output(S1*gray_id_1 | S2*gray_id_2 | S3*gray_id_3 | S4*gray_id_4 | S5*gray_id_5 | S6*gray_id_6 | S7*gray_id_7);
  Serial.print("state: ");
  Serial.println(S1*gray_id_1 | S2*gray_id_2 | S3*gray_id_3 | S4*gray_id_4 | S5*gray_id_5 | S6*gray_id_6 | S7*gray_id_7);
  Serial.print("output: ");
  Serial.println(output);

  // 在mot函数里已经自动约束了。
  motor.mot(255+output, 255-output);
}

/**************************************************************/

void readsensor() 
{
  S1 = digitalRead(sensor1);
  S2 = digitalRead(sensor2);
  S3 = digitalRead(sensor3);
  S4 = digitalRead(sensor4);
  S5 = digitalRead(sensor5);
  S6 = digitalRead(sensor6);
  S7 = digitalRead(sensor7);
}

MotorState checkState(unsigned char state)
{
    static MotorState pre_state;
    switch (state)
    {
        case G0011100://345
        case G0111110://23456
        case G0111100://2345
        case G0011110://3456
            pre_state =  _FAST_STRAIGHT_;
            break;
        case G0011000://34
            pre_state =  _SLOW_LEFT_;
            break;
        case G0001100://45
            pre_state =  _SLOW_RIGHT_;
            break;
        case G0111000://234
            pre_state =  _FAST_LEFT_;
            break;
        case G0001110://456
            pre_state =  _FAST_RIGHT_;
            break;
        case G1100000://12
            pre_state =  _FAST_LEFT_;
            break;
        case G0000011://67
            pre_state =  _FAST_RIGHT_;
            break;
        case G0110000://23
            pre_state =  _FAST_LEFT_;
            break;
        case G0000110://56
             pre_state =  _FAST_RIGHT_;
             break;
        case G1111111://1234567
            pre_state =  _FAST_STRAIGHT_;
            break;
        default:
            break;
    }
    return pre_state;
}

void tracking()
{
  // first readsensor
  readsensor();
  MotorState state1 = checkState(S1*gray_id_1 | S2*gray_id_2 | S3*gray_id_3 | S4*gray_id_4 | S5*gray_id_5 | S6*gray_id_6 | S7*gray_id_7);
  delay(t);
  readsensor();
  MotorState state2 = checkState(S1*gray_id_1 | S2*gray_id_2 | S3*gray_id_3 | S4*gray_id_4 | S5*gray_id_5 | S6*gray_id_6 | S7*gray_id_7);
  delay(t);
  readsensor();
  MotorState state3 = checkState(S1*gray_id_1 | S2*gray_id_2 | S3*gray_id_3 | S4*gray_id_4 | S5*gray_id_5 | S6*gray_id_6 | S7*gray_id_7);

   if (state1 == state2 && state2 == state3)
     motor.changeState(state1);
}

//超声波测距
void measure()
{
  digitalWrite(leftTrigPin, LOW);
  delay(2);
  digitalWrite(leftTrigPin, HIGH);
  delay(10);
  digitalWrite(leftTrigPin, LOW);
  float left_distance = pulseIn(leftEchoPin, HIGH, 6000) * 0.017;
  left_distance = left_distance > 0 ? left_distance : 100;
  Serial.print("left diatance:");
  Serial.println(left_distance);
  digitalWrite(rightTrigPin, LOW);
  delay(2);
  digitalWrite(rightTrigPin, HIGH);
  delay(10);
  digitalWrite(rightTrigPin, LOW);
  float right_distance = pulseIn(rightEchoPin, HIGH, 6000) * 0.017;
  right_distance = right_distance > 0 ? right_distance : 100;
  Serial.print("right diatance:");
  Serial.println(right_distance);
  distance = left_distance < right_distance ? left_distance : right_distance;
  Serial.print("diatance:");
  Serial.println(distance);
  if (near_Barrier == 1 && distance < 20 && distance>15){
      delay(t);
      if (near_Barrier == 1 && distance < 20 && distance>15){
        delay(t);
        if (near_Barrier == 1 && distance < 20 && distance>15){
          avoid_Barrier();
        }
      }
  }
}

//避障
void avoid_Barrier()
{
	motor.changeState(_LEFT_ROTATION_);
	delay(600);
	motor.changeState(_FAST_STRAIGHT_);
	delay(500);
	motor.changeState(_RIGHT_ROTATION_);
	delay(600);
  motor.changeState(_FAST_STRAIGHT_);
  delay(850);
  motor.changeState(_RIGHT_ROTATION_);
  delay(600);
  motor.changeState(_FAST_STRAIGHT_);
  delay(500);
  motor.changeState(_LEFT_ROTATION_);
  delay(600);
  line = 6;   
  next_flag = 0;   //避障完成后重置line,next_flag
	near_Barrier = 0;
}

void change_flag()
{
  flag = 1;
}

void change_line()
{
  static unsigned long time=millis();
  unsigned long current_time = millis();
  unsigned long dec = current_time - time;
  if (dec >= 666 && stop_five_sec == 0)
  {
    line--;
    line = line > 0 ? line : 0;
    Serial.println(line);
    time = current_time;
  }
}

//当line为0时触发停止，且是通过中间的进行触发，此处主要
void stop()
{
  if (line == 0) {
    if (next_flag == 0)  //下一步停止
    {
      Serial.println("stop");
      stop_five_sec = 1;
      line = next_line;
      next_flag = 1;
    }
    else if (next_flag == 1)  //下一步避障
    {
      near_Barrier = 1;
    }
  }
}


void setup() {
	Serial.begin(9600);
	pinMode(leftTrigPin, OUTPUT);
	pinMode(rightTrigPin, OUTPUT);
	pinMode(leftEchoPin, INPUT);
	pinMode(rightEchoPin, INPUT);
	pinMode(grey_a, INPUT);
	pinMode(grey_b, INPUT);
  attachInterrupt(4, change_flag, RISING);
  attachInterrupt(5, stop, FALLING);
}


void loop() {
  tracking_pid();
  // tracking();
  if (stop_five_sec == 1)
  {
    motor.changeState(_NORMAL_STRAIGHT_);
    delay(120);
    motor.changeState(_STOP_);
    delay(5000);
    stop_five_sec = 0;
  }
  if (flag == 1)
  {
    change_line();
  }
  if(near_Barrier)
  {
    measure();
  }
  measure();
//  delay(3000);
}
