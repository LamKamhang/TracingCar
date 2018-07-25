#include "PID.h"
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// global variables
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


//||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// main funcitons
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void setup()
{
    Input = calerror();
    Setpoint = 0;
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

void loop()
{
    Input = calerror();

    double gap = abs(Setpoint-Input); //distance away from setpoint
    if (gap < 10)
    {  //we're close to setpoint, use conservative tuning parameters
        myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
        //we're far from setpoint, use aggressive tuning parameters
        myPID.SetTunings(aggKp, aggKi, aggKd);
    }

    myPID.Compute();
    // Output can be used.
    MotorAdjust(Output);
}