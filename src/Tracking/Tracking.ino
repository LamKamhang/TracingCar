#include "Motor.h"

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
const int grey_c = 17;

const int BLACK = 1;
const int WHITE = 0;		//定义颜色

int S1, S2, S3, S4, S5, S6, S7;  //七路巡线返回值
const int t = 1;

int basic_velocity;		//定义正常速度
int current_velocity;		//定义当前速度
float distance;				//定义最小距离变量
int near_Barrier = 0;			//定义避障状态变量
int stop_five_sec = 0;		//定义静止函数
int line;
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


void readsensor() {
  S1 = digitalRead(sensor1);
  S2 = digitalRead(sensor2);
  S3 = digitalRead(sensor3);
  S4 = digitalRead(sensor4);
  S5 = digitalRead(sensor5);
  S6 = digitalRead(sensor6);
  S7 = digitalRead(sensor7);
}

// void mot(int left_velocity,int right_velocity) {
//   digitalWrite(left_front_wheel2, LOW);
//   analogWrite(left_front_wheel1, left_velocity);  //前左轮正转
//   digitalWrite(left_behind_wheel1, LOW);
//   analogWrite(left_behind_wheel2, left_velocity); //后左轮正转
//   digitalWrite(right_front_wheel1, LOW);
//   analogWrite(right_front_wheel2, right_velocity);//前右轮正转
//   digitalWrite(right_behind_wheel2, LOW);
//   analogWrite(right_behind_wheel1, right_velocity);//后右轮正转
// }

// void mot_reverse(int left_velocity, int right_velocity) {
// 	digitalWrite(left_front_wheel1, LOW);
// 	analogWrite(left_front_wheel2, left_velocity);  //前左轮反转
// 	digitalWrite(left_behind_wheel2, LOW);
// 	analogWrite(left_behind_wheel1, left_velocity); //后左轮反转
// 	digitalWrite(right_front_wheel2, LOW);
// 	analogWrite(right_front_wheel1, right_velocity);//前右轮反转
// 	digitalWrite(right_behind_wheel1, LOW);
// 	analogWrite(right_behind_wheel2, right_velocity);//后右轮反转
// }

// void straight_test() {
//   readsensor();
//   /*if (ramp==0 && avo == 1 && SL == BLACK && (S3 == BLACK || S2 == BLACK || S4 == BLACK)) {
//   delay(t);
//   readsensor;
//   if (SL == BLACK) {
//   delay(t);
//   readsensor;
//   if (SL == BLACK) {
//   ramp = 1;
//   }
//   }
//   }
//   */
//   if (S3 == BLACK && S4 == BLACK && S5 == BLACK && S2 == WHITE && S6 == WHITE) {
//     delay(t);
//     readsensor();
//     if (S3 == BLACK && S4 == BLACK && S5 == BLACK && S2 == WHITE && S6 == WHITE) {
//       delay(t);
//       readsensor();
//       if (S3 == BLACK && S4 == BLACK && S5 == BLACK && S2 == WHITE && S6 == WHITE) {
//         mot(current_velocity, current_velocity);
//       }
//     }
//   }

//   else if (S3 == BLACK && S4 == BLACK && S5 == WHITE) {
//     delay(t);
//     readsensor();
//     if (S3 == BLACK && S4 == BLACK && S5 == WHITE) {
//       delay(t);
//       readsensor();
//       if (S3 == BLACK && S4 == BLACK && S5 == WHITE) {
//         mot((current_velocity-100>0? current_velocity:0), current_velocity);
//       }
//     }
//   }
  

//   else if (S4 == BLACK && S5 == BLACK && S3 == WHITE) {
//     delay(t);
//     readsensor();
//     if (S4 == BLACK && S5 == BLACK && S3 == WHITE) {
//       delay(t);
//       readsensor();
//       if (S4 == BLACK && S5 == BLACK && S3 == WHITE) {
//         mot(current_velocity, (current_velocity - 100>0 ? current_velocity : 0));
//       }
//     }
//   }

//   else if (S2 == BLACK && S3 == BLACK && S4 == BLACK && S1 == WHITE && S5 == WHITE) {
//     delay(t);
//     readsensor();
//     if (S2 == BLACK && S3 == BLACK && S4 == BLACK && S1 == WHITE && S5 == WHITE) {
//       delay(t);
//       readsensor();
//       if (S2 == BLACK && S3 == BLACK && S4 == BLACK && S1 == WHITE && S5 == WHITE) {
//         mot((current_velocity - 150>0 ? current_velocity : 0), current_velocity);
//       }
//     }
//   }

//   else if (S4 == BLACK && S5 == BLACK && S6 == BLACK && S3 == WHITE && S7 == WHITE) {
//     delay(t);
//     readsensor();
//     if (S4 == BLACK && S5 == BLACK && S6 == BLACK && S3 == WHITE && S7 == WHITE) {
//       delay(t);
//       readsensor();
//       if (S4 == BLACK && S5 == BLACK && S6 == BLACK && S3 == WHITE && S7 == WHITE) {
//         mot(current_velocity, (current_velocity - 150>0 ? current_velocity : 0));
//       }
//     }
//   }

//    else if (S2 == BLACK && S3 == BLACK && S4 == WHITE) {
//     delay(t);
//     readsensor();
//     if (S2 == BLACK && S3 == BLACK && S4 == WHITE) {
//       delay(t);
//       readsensor();
//       if (S2 == BLACK && S3 == BLACK && S4 == WHITE) {
//         mot(0, current_velocity);
//       }
//     }
//   }

//    else if (S5 == BLACK && S6 == BLACK && S4 == WHITE) {
//     delay(t);
//     readsensor();
//     if (S5 == BLACK && S6 == BLACK && S4 == WHITE) {
//       delay(t);
//       readsensor();
//       if (S5 == BLACK && S6 == BLACK && S4 == WHITE) {
//         mot(current_velocity, 0);
//       }
//     }
//   }

//    else if (S1 == BLACK && S2 == BLACK && S3 == BLACK) {
//     delay(t);
//     readsensor();
//     if (S1 == BLACK && S2 == BLACK && S3 == BLACK) {
//       delay(t);
//       readsensor();
//       if (S1 == BLACK && S2 == BLACK && S3 == BLACK) {
//         mot(0, current_velocity);
//       }
//     }
//   }
  
//    else if (S5 == BLACK && S6 == BLACK && S7 == BLACK) {
//     delay(t);
//     readsensor();
//     if (S5 == BLACK && S6 == BLACK && S7 == BLACK) {
//       delay(t);
//       readsensor();
//       if (S5 == BLACK && S6 == BLACK && S7 == BLACK) {
//         mot(current_velocity, 0);
//       }
//     }
//   }
  
//    else if (S1 == BLACK && S2 == BLACK && S3 == WHITE) {
//     delay(t);
//     readsensor();
//     if (S1 == BLACK && S2 == BLACK && S3 == WHITE) {
//       delay(t);
//       readsensor();
//       if (S1 == BLACK && S2 == BLACK && S3 == WHITE) {
//         mot(0, current_velocity);
//       }
//     }
//   }
  

//   else if (S6 == BLACK && S7 == BLACK && S5 == WHITE) {
//     delay(t);
//     readsensor();
//     if (S6 == BLACK && S7 == BLACK && S5 == WHITE) {
//       delay(t);
//       readsensor();
//       if (S6 == BLACK && S7 == BLACK && S5 == WHITE) {
//         mot(current_velocity, 0);
//       }
//     }
//   }

//   else if (S2 == WHITE && S1 == BLACK) {
//     delay(t);
//     readsensor();
//     if (S2 == WHITE && S1 == BLACK) {
//       delay(t);
//       readsensor();
//       if (S2 == WHITE && S1 == BLACK) {
//         mot(0, current_velocity);
//       }
//     }
//   }

//   else if (S6 == WHITE && S7 == BLACK) {
//     delay(t);
//     readsensor();
//     if (S6 == WHITE && S7 == BLACK) {
//       delay(t);
//       readsensor();
//       if (S6 == WHITE && S7 == BLACK) {
//         mot(current_velocity, 0);
//       }
//     }
//   }

//   else if (S2 == BLACK) {
//     delay(t);
//     readsensor();
//     if (S2 == BLACK) {
//       delay(t);
//       readsensor();
//       if (S2 == BLACK) {
//         mot(0, current_velocity);
//       }
//     }
//   }

  
//   else if (S6 == BLACK) {
//     delay(t);
//     readsensor();
//     if (S6 == BLACK) {
//       delay(t);
//       readsensor();
//       if (S6 == BLACK) {
//         mot(current_velocity, 0);
//       }
//     }
//   }

//   /*
//   else if(S1 == WHITE && S2 == WHITE && S3 == WHITE && S5 == WHITE && S5 == WHITE && S6 == WHITE && S7 == WHITE)
//   {
// 	  delay(10*t);
// 	  readsensor();
// 	  if (S1 == WHITE && S2 == WHITE && S3 == WHITE && S5 == WHITE &&S5 == WHITE && S6 == WHITE && S7 == WHITE) {
// 		  delay(10*t);
// 		  readsensor();
// 		  if (S1 == WHITE && S2 == WHITE && S3 == WHITE && S5 == WHITE && S5 == WHITE && S6 == WHITE && S7 == WHITE) {
// 			  mot(0, 0);
// 		  }
// 	  }
//   }
//   */
// }

const unsigned char gray_id_1 = 1;
const unsigned char gray_id_2 = 1 << 1;
const unsigned char gray_id_3 = 1 << 2;
const unsigned char gray_id_4 = 1 << 3;
const unsigned char gray_id_5 = 1 << 4;
const unsigned char gray_id_6 = 1 << 5;
const unsigned char gray_id_7 = 1 << 6;

int checkState(unsigned char state)
{
    static int pre_state;
    switch (state)
    {
        case (28)://345
        case (62)://23456
        case (30)://2345
        case (60)://3456
            pre_state =  _FAST_STRAIGHT_;
            break;
        case (12)://34
            pre_state =  _SLOW_LEFT_;
            break;
        case (24)://45
            pre_state =  _SLOW_RIGHT_;
            break;
        case (14)://234
            pre_state =  _FAST_LEFT_;
            break;
        case (56)://456
            pre_state =  _FAST_RIGHT_;
            break;
        case (3)://12
            pre_state =  _FAST_LEFT_
            ;
            break;
        case (96)://67
            pre_state =  _FAST_RIGHT_;
            break;
        case (6)://23
            pre_state =  _FAST_LEFT_;
            break;
        case (48)://56
             pre_state =  _FAST_RIGHT_;
             break;
        case (127)://1234567
            pre_state =  _FAST_STRAIGHT_;
            break;
        default:
            break;
    }
    return pre_state;
}

void straight_test()
{
  // first readsensor
  readsensor();
  int state1 = checkState(S1*gray_id_1 | S2*gray_id_2 | S3*gray_id_3 | S4*gray_id_4 | S5*gray_id_5 | S6*gray_id_6 | S7*gray_id_7);
  delay(t);
  readsensor();
  int state2 = checkState(S1*gray_id_1 | S2*gray_id_2 | S3*gray_id_3 | S4*gray_id_4 | S5*gray_id_5 | S6*gray_id_6 | S7*gray_id_7);
  delay(t);
  readsensor();
  int state3 = checkState(S1*gray_id_1 | S2*gray_id_2 | S3*gray_id_3 | S4*gray_id_4 | S5*gray_id_5 | S6*gray_id_6 | S7*gray_id_7);

   if (state1 == state2 && state2 == state3)
     motor.changeState(state1);
}

void barrier()
{
	digitalWrite(leftTrigPin, LOW);
	delay(2);
	digitalWrite(leftTrigPin, HIGH);
	delay(10);
	digitalWrite(leftTrigPin, LOW);
	float distanceL = pulseIn(leftEchoPin, HIGH, 6000)*0.017;
	digitalWrite(rightTrigPin, LOW);
	delay(2);
	digitalWrite(rightTrigPin, HIGH);
	delay(10);
	digitalWrite(rightTrigPin, LOW);
	float distanceR = pulseIn(rightEchoPin, HIGH, 6000)*0.017;
  distance = distanceL < distanceR ? distanceR : distanceL;
  Serial.println(distance);
	//if (near_Barrier == 1 && distance < 20 && distance>5)
  if (distance <= 20 && distance >= 5)
		avoid_Barrier();
}

void avoid_Barrier()
{
	motor.changeState(_LEFT_ROTATION_);
	delay(200);
	motor.changeState(_FAST_STRAIGHT_);
	delay(1000);
	motor.changeState(_RIGHT_ROTATION_);
	delay(400);
	motor.changeState(_FAST_STRAIGHT_);
	near_Barrier = 0;
}

void count()
{
	line--;
	Serial.print("line:");
	Serial.println(line);
	if (line == 0) {
		Serial.println("slow");
		current_velocity = 100;
	}
}

void stop()
{
	if (line == 0) {
		Serial.println("stop");
		stop_five_sec = 1;
	}
}

void setup() {
	Serial.begin(9600);
	line = 2;
	basic_velocity = 255;
	current_velocity = basic_velocity;
	pinMode(leftTrigPin, OUTPUT);
	pinMode(rightTrigPin, OUTPUT);
	pinMode(leftEchoPin, INPUT);
	pinMode(rightEchoPin, INPUT);
	pinMode(grey_a, INPUT);
	pinMode(grey_b, INPUT);
	pinMode(grey_c, INPUT);
//	attachInterrupt(4, count, RISING);
//	attachInterrupt(5, stop, FALLING);
}


void loop() {
  //motor.changeState(_FAST_STRAIGHT_);
	straight_test();
//	if (stop_five_sec == 1)
//	{
//		mot_reverse(200, 200);
//		delay(120);
//		mot(0, 0);
//		delay(5000);
//		current_velocity = basic_velocity;		//停止后静止5秒重新启动
//		straight_test();
//		line = 2;
//		stop_five_sec = 0;
//	}
//	if (pulseIn(19, HIGH) / 1000 > 300)
//	{
//		count();
//	}
  barrier();
}
