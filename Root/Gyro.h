#pragma once
#include <math.h>
#include <wire.h>
#include "MPU6050.h"



class Gyro
{
public:
    Gyro();
    float getRelativeYaw();
	void setInitYaw(float delta);
    float getYaw();
	void init();
	void run();
private:
	void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);
private:
    MPU6050lib mpu;
    float initYaw;
    float aRes, gRes; // scale resolutions per LSB for the sensors
    int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
    float ax, ay, az;       // Stores the real accel value in g's
    int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
    float gx, gy, gz;       // Stores the real gyro value in degrees per seconds
    float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
    int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
    float temperature;
    float SelfTest[6];
    float q[4];            // vector to hold quaternion
    uint32_t delt_t; // used to control display output rate
    uint32_t count;  // used to control display output rate
    float pitch, yaw, roll;
    // parameters for 6 DoF sensor fusion calculations
    float GyroMeasError;     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    float beta;  // compute beta
    float GyroMeasDrift;      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float zeta;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    float deltat;                              // integration interval for both filter schemes
    uint32_t lastUpdate, firstUpdate;         // used to calculate integration interval
    uint32_t Now;
};