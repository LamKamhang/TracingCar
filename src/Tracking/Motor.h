#pragma once
#include <Arduino.h>
/*
    left_front_wheel1 = 4;
    left_front_wheel2 = 5;
    right_front_wheel1 = 10;
    right_front_wheel2 = 11;
    left_behind_wheel1 = 8;
    left_behind_wheel2 = 9;
    right_behind_wheel1 = 7;
    right_behind_wheel2 = 6;
*/
enum MotorState {
	_FAST_STRAIGHT_,
	_NORMAL_STRAIGHT_,
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
        int left_front_wheel1,
        int left_front_wheel2,
        int right_front_wheel1,
        int right_front_wheel2,
        int left_behind_wheel1,
        int left_behind_wheel2,
        int right_behind_wheel1,
        int right_behind_wheel2
    );
    void init();
    void changeState(MotorState state);
	void mot(int left_velocity,int right_velocity);
private:
    
    void fast_straight();
    void normal_straight();
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
    int left_front_wheel1;
    int left_front_wheel2;
    int right_front_wheel1;
    int right_front_wheel2;
    int left_behind_wheel1;
    int left_behind_wheel2;
    int right_behind_wheel1;
    int right_behind_wheel2;
};

