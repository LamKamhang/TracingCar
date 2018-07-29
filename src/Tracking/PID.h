#pragma once


#define _STABLE_              0   
#define _SLIGHT_LEFT_         -1  
#define _SLIGHT_RIGHT_        1   
#define _MODERATE_LEFT_       -3 
#define _MODERATE_RIGHT_      3  
#define _SERIOUS_LEFT_        -5  
#define _SERIOUS_RIGHT_       5   
#define _EXTREME_LEFT_        -7  
#define _EXTREME_RIGHT_       7   


class PID
{
public:
    PID();
    PID(double Kp, double Ki, double Kd);
    int get_output(int state);

private:
    int cal_deviation(int state);

private:
    double Kp, Ki, Kd;
    int integral;
    int preInput;
};