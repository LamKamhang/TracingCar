#pragma once
#include <Arduino.h>
/*
    //定义车轮驱动接口
    left_wheel1 = 9;
    left_wheel2 = 10;
    left_wheelPWM = 8;
    right_wheel1 = 5;
    right_wheel2 = 6;
    right_wheelPWM = 4;
*/
enum MotorState {
	_FAST_STRAIGHT_,
	_NORMAL_STRAIGHT_,
    _NORMAL_BACKWARD_,
	_STOP_,
	_FAST_LEFT_,
	_NORMAL_LEFT_,
	_SLOW_LEFT_,
	_FAST_RIGHT_,
	_NORMAL_RIGHT_,
	_SLOW_RIGHT_,
	_LEFT_ROTATION_,
	_RIGHT_ROTATION_,
};

class Motor
{
public:
    Motor(
        int left_wheelPWM,
		int left_wheel1,
        int left_wheel2,
		int right_wheelPWM,
        int right_wheel1,
        int right_wheel2
    );
    void init();
    void changeState(MotorState state);
	void mot(int left_velocity,int right_velocity);
private:    
    int constrain_(int velocity);
    void fast_straight();
    void normal_straight();
    void normal_backward();
    void fast_left();
    void fast_right();
    void normal_left();
    void normal_right();
    void slow_left();
    void slow_right();
    void stop();
	void rotationL();
	void rotationR();

private:
    int left_wheel1;
    int left_wheel2;
	int left_wheelPWM;
    int right_wheel1;
    int right_wheel2;
	int right_wheelPWM;
};

