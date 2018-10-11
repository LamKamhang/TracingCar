#pragma once

// 特殊状态的标记
//#define _LOW_START_FLAG_
#define _STOP_1_FLAG_
#define _STOP_1_READY_TIME_ 3500

#define _STOP_2_FLAG_
#define _STOP_2_READY_TIME_ 1000

#define _ADJUST_TIME 2000
#define _ADJUST_TIME2 2000
#define _BARRIER_TIME 1000
#define _MESSURE_TIME 30	
#define _SOME_READY_TIME 5000

// 夹取调整的距离时间
#define _CATCH_SPEED        50
#define _CATCH_BACK_TIME    1500
#define _CATCH_FORWARD_TIME 700
#define _PLACE_BACK_TIME    1010
#define _PLACE_FORWARD_TIME 575

//定义状态
#define G1111	15
#define G0111	7
#define G1011	11
#define G1110	14
#define G1101	13
#define G1001	9
#define G0110	6
#define G1010	10
#define G0101	5
#define G0100	4
#define G1000	8
#define G1100	12

//抓取个数
#define _BLOCK_NUMBER_ 3

//振荡停止
#define wave_times  2
#define wave_velocity  100

//避障时间
#define _ROTATION_TIME 900
#define _STRAIGHT_TIME 

//定义七路灰度传感器接口
#define sensor1 30
#define sensor2 32
#define sensor3 34
#define sensor4 36
#define sensor5 38
#define sensor6 40
#define sensor7 42

//定义黑白
#define BLACK 1
#define WHITE 0

//定义车轮驱动接口
#define left_wheel1 9
#define left_wheel2 10
#define left_wheelPWM 8
#define right_wheel1 5
#define right_wheel2 6
#define right_wheelPWM 4

//定义超声波探测引脚
#define leftTrigPin 23
#define leftEchoPin 22
#define rightTrigPin 44
#define rightEchoPin 45

//定义三路灰度传感器
#define grey_a 26		//右前方传感器
#define grey_b 28		//右后方传感器
#define grey_c 24       //左方传感器
#define grey_l 20		
#define grey_r 18		//后方传感器
