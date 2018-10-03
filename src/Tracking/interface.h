#pragma once

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
#define left_wheel1 = 9;
#define left_wheel2 = 10;
#define left_wheelPWM = 8;
#define right_wheel1 = 5;
#define right_wheel2 = 6;
#define right_wheelPWM = 4;

//定义超声波探测引脚
#define leftTrigPin = 23;
#define leftEchoPin = 22;
#define rightTrigPin = 44;
#define rightEchoPin = 45;

//定义三路灰度传感器
#define grey_a = 18;		//前方传感器
#define grey_b = 19;		//中央传感器
#define grey_c = 20;		//后方传感器