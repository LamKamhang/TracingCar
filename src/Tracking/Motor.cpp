#include "Motor.h"


Motor::Motor(
    int left_front_wheel1,
    int left_front_wheel2,
    int right_front_wheel1,
    int right_front_wheel2,
    int left_behind_wheel1,
    int left_behind_wheel2,
    int right_behind_wheel1,
    int right_behind_wheel2
    )
    : left_front_wheel1(left_front_wheel1)
    , left_front_wheel2(left_front_wheel2)
    , right_front_wheel1(right_front_wheel1)
    , right_front_wheel2(right_front_wheel2)
    , left_behind_wheel1(left_behind_wheel1)
    , left_behind_wheel2(left_behind_wheel2)
    , right_behind_wheel1(right_behind_wheel1)
    , right_behind_wheel2(right_behind_wheel2)
{}

void Motor::init()
{
    pinMode(left_front_wheel1, OUTPUT);
    pinMode(left_front_wheel2, OUTPUT);
    pinMode(right_front_wheel1, OUTPUT);
    pinMode(right_front_wheel2, OUTPUT);
    pinMode(left_behind_wheel1, OUTPUT);
    pinMode(left_behind_wheel2, OUTPUT);
    pinMode(right_behind_wheel1, OUTPUT);
    pinMode(right_behind_wheel2, OUTPUT);
}

void Motor::changeState(MotorState state)
{
    switch(state)
    {
    case _FAST_STRAIGHT_:   fast_straight();        break;
    case _NORMAL_STRAIGHT_: normal_straight();      break;
    case _NORMAL_BACKWARD_: normal_backward();      break;
    case _STOP_:            stop();                 break;
    case _FAST_LEFT_:       fast_left();            break;
    case _NORMAL_LEFT_:     normal_left();          break;
    case _FAST_RIGHT_:      fast_right();           break;
    case _NORMAL_RIGHT_:    normal_right();         break;
    case _SLOW_LEFT_:       slow_left();            break;
    case _SLOW_RIGHT_:      slow_right();           break;
	case _LEFT_ROTATION_:	rotationL();			break;
	case _RIGHT_ROTATION_:	rotationR();			break;
    default:                    break;
    }
}

void Motor::fast_straight()
{
    mot(255, 255);
}

void Motor::normal_straight()
{
    mot(200, 200);
}

void Motor::normal_backward()
{
    mot(-200, -200);
}

void Motor::fast_left()
{
    mot(-100, 255);
}

void Motor::fast_right()
{
    mot(255, -100);
}

void Motor::normal_left()
{
    mot(100, 255);
}

void Motor::normal_right()
{
    mot(255, 100);
}

void Motor::stop()
{
    mot(0, 0);
}

void Motor::slow_left()
{
    mot(150, 255);
}

void Motor::slow_right()
{
    mot(255, 150);
}

void Motor::rotationL()
{
    mot(-255, 255);
}
void Motor::rotationR()
{
    mot(255, -255);
}

void Motor::mot(int left_velocity,int right_velocity) {
    left_velocity = constrain_(left_velocity);
    right_velocity = constrain_(right_velocity);

	if (left_velocity >= 0 && right_velocity >= 0)
	{
        digitalWrite(left_front_wheel2, LOW);
        analogWrite(left_front_wheel1, left_velocity);  //前左轮正转
        digitalWrite(left_behind_wheel1, LOW);
        analogWrite(left_behind_wheel2, left_velocity); //后左轮正转
        digitalWrite(right_front_wheel1, LOW);
        analogWrite(right_front_wheel2, right_velocity);//前右轮正转
        digitalWrite(right_behind_wheel2, LOW);
        analogWrite(right_behind_wheel1, right_velocity);//后右轮正转
	}
	else if (left_velocity >= 0 && right_velocity <= 0)
	{
        digitalWrite(left_front_wheel2, LOW);
        analogWrite(left_front_wheel1, left_velocity);  //前左轮正转
        digitalWrite(left_behind_wheel1, LOW);
        analogWrite(left_behind_wheel2, left_velocity); //后左轮正转
        digitalWrite(right_front_wheel2, LOW);
        analogWrite(right_front_wheel1, -right_velocity);//前右轮反转
        digitalWrite(right_behind_wheel1, LOW);
        analogWrite(right_behind_wheel2, -right_velocity);//后右轮反转
	}
	else if (left_velocity <= 0 && right_velocity >= 0)
	{
        digitalWrite(left_front_wheel1, LOW);
        analogWrite(left_front_wheel2, -left_velocity);  //前左轮反转
        digitalWrite(left_behind_wheel2, LOW);
        analogWrite(left_behind_wheel1, -left_velocity); //后左轮反转
        digitalWrite(right_front_wheel1, LOW);
        analogWrite(right_front_wheel2, right_velocity);//前右轮正转
        digitalWrite(right_behind_wheel2, LOW);
        analogWrite(right_behind_wheel1, right_velocity);//后右轮正转
	}
	else
	{
        digitalWrite(left_front_wheel1, LOW);
        analogWrite(left_front_wheel2, -left_velocity);  //前左轮反转
        digitalWrite(left_behind_wheel2, LOW);
        analogWrite(left_behind_wheel1, -left_velocity); //后左轮反转
        digitalWrite(right_front_wheel2, LOW);
        analogWrite(right_front_wheel1, -right_velocity);//前右轮反转
        digitalWrite(right_behind_wheel1, LOW);
        analogWrite(right_behind_wheel2, -right_velocity);//后右轮反转
	}
}

int Motor::constrain_(int velocity)
{
    if (velocity < -255)
        return -255;
    else if (velocity > 255)
        return 255;
    else
        return velocity;
}