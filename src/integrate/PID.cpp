#include "PID.h"
#include "util.hpp"

PID::PID()
    :   Kp(0)
    ,   Ki(0)
    ,   Kd(0)
    ,   integral(0)
    ,   preInput(0)
{}

PID::PID(double Kp, double Ki, double Kd)
    :   Kp(Kp)
    ,   Ki(Ki)
    ,   Kd(Kd)
    ,   integral(0)
    ,   preInput(0)
{}

int PID::get_output(int state)
{
    int input = cal_deviation(state);

    integral += input;
    int diff = input - preInput;

    preInput = input;
    return (int)(Kp*input + Ki*integral + Kd*diff);
}

void PID::reset(void)
{
    integral = 0;
    preInput = 0;
}

int PID::cal_deviation(int state)
{
    int input = preInput;
    switch (state)
    {
        // 在中间
        case G0001000:
        case G0011100:
        case G0111110://23456
        case G1111111:
            input =  _STABLE_;
            break;
        // 稍微偏左
        case G0011000://34
        case G0111100:
        case G1111110:
            input =  _SLIGHT_LEFT_;
            break;
        // 稍微偏右
        case G0001100://45
        case G0011110:
        case G0111111:
            input =  _SLIGHT_RIGHT_;
            break;
        // 正常偏左：
        case G0111000://234
        case G1111100:
            input =  _MODERATE_LEFT_;
            break;
        // 正常偏右：
        case G0001110://456
        case G0011111:
            input =  _MODERATE_RIGHT_;
            break;
        // 严重偏左：
        case G0110000://234
        case G1110000:
        case G1111000:
            input =  _SERIOUS_LEFT_;
            break;
        // 严重偏右
        case G0000110:
        case G0000111:
        case G0001111://67
            input =  _SERIOUS_RIGHT_;
            break;
        // 极度偏左
        case G0100000://23
        case G1100000:
        case G1000000:
            input =  _EXTREME_LEFT_;
            break;
        // 极度偏右
        case G0000010://56
        case G0000011:
        case G0000001:
             input =  _EXTREME_RIGHT_;
             break;
        default:
            break;
    }
    return input;
}