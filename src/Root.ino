#include <Wire.h>
#include "MPU6050.h"

//定义陀螺仪相关变量
MPU6050lib mpu;
float aRes, gRes; // scale resolutions per LSB for the sensors
				  // Pin definitions
int intPin = 2;  // These can be changed, 2 and 3 are the Arduinos ext int pins
#define blinkPin 13  // Blink LED on Teensy or Pro Mini when updating
boolean blinkOn = false;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;       // Stores the real accel value in g's
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;       // Stores the real gyro value in degrees per seconds
float gyroBias[3] = { 0, 0, 0 }, accelBias[3] = { 0, 0, 0 }; // Bias corrections for gyro and accelerometer
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };            // vector to hold quaternion
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate
float pitch, yaw, roll;// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
//定义超声波探测引脚
const int leftTrigPin = 23;
const int leftEchoPin = 22;
const int rightTrigPin = 44;
const int rightEchoPin = 45;		
//定义三路灰度传感器
const int grey_a = 19;
const int grey_b = 18;
const int grey_c = 17;			
//定义七路灰度传感器
const int grey_1 = 30;
const int grey_2 = 32;
const int grey_3 = 34;
const int grey_4 = 36;
const int grey_5 = 38;
const int grey_6 = 40;
const int grey_7 = 42;			

float distance;				//定义最小距离变量
int near_Barrier=0;			//定义避障状态变量

void barrier()
{
	digitalWrite(leftTrigPin, LOW);
	delay(2);
	digitalWrite(leftTrigPin, HIGH);
	delay(10);
	digitalWrite(leftTrigPin, LOW);
	distance = pulseIn(leftEchoPin, HIGH,6000)*0.017;
	Serial.print(distance);
	Serial.println("cm");
	digitalWrite(rightTrigPin, LOW);
	delay(2);
	digitalWrite(rightTrigPin, HIGH);
	delay(10);
	digitalWrite(rightTrigPin, LOW);
	distance = pulseIn(rightEchoPin, HIGH,6000)*0.017;
	Serial.print(distance);
	Serial.println("cm");
	if (near_Barrier == 1 && distance < 20 && distance>5)
		avoid_Barrier();
}

void avoid_Barrier()
{

	near_Barrier = 0;
}

void interrupt()
{
	detachInterrupt(5);
	attachInterrupt(4, slow, RISING);		//利用21号口作为中断
	attachInterrupt(5, stop, FALLING);		//利用20号口作为中断
}

void slow()
{
	
}

void stop()
{
	
}

void back()
{
	
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
	float norm;                                               // vector norm
	float f1, f2, f3;                                         // objetive funcyion elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float qDot1, qDot2, qDot3, qDot4;
	float hatDot1, hatDot2, hatDot3, hatDot4;
	float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error
	// Auxiliary variables to avoid repeated arithmetic
	float _halfq1 = 0.5f * q1;
	float _halfq2 = 0.5f * q2;
	float _halfq3 = 0.5f * q3;
	float _halfq4 = 0.5f * q4;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;
	// Compute the objective function and Jacobian
	f1 = _2q2 * q4 - _2q1 * q3 - ax;
	f2 = _2q1 * q2 + _2q3 * q4 - ay;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
	J_11or24 = _2q3;
	J_12or23 = _2q4;
	J_13or22 = _2q1;
	J_14or21 = _2q2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;
	// Compute the gradient (matrix multiplication)
	hatDot1 = J_14or21 * f2 - J_11or24 * f1;
	hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
	hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
	hatDot4 = J_14or21 * f1 + J_11or24 * f2;
	// Normalize the gradient
	norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
	hatDot1 /= norm;
	hatDot2 /= norm;
	hatDot3 /= norm;
	hatDot4 /= norm;
	// Compute estimated gyroscope biases
	gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
	gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
	gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
	// Compute and remove gyroscope biases
	gbiasx += gerrx * deltat * zeta;
	gbiasy += gerry * deltat * zeta;
	gbiasz += gerrz * deltat * zeta;
	gx -= gbiasx;
	gy -= gbiasy;
	gz -= gbiasz;
	// Compute the quaternion derivative
	qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
	qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
	qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
	qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;
	// Compute then integrate estimated quaternion derivative
	q1 += (qDot1 - (beta * hatDot1)) * deltat;
	q2 += (qDot2 - (beta * hatDot2)) * deltat;
	q3 += (qDot3 - (beta * hatDot3)) * deltat;
	q4 += (qDot4 - (beta * hatDot4)) * deltat;
	// Normalize the quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void ypr()
{
	// If data ready bit set, all data registers have new data
	if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { // check if data ready interrupt
		mpu.readAccelData(accelCount);  // Read the x/y/z adc values
		aRes = mpu.getAres();
		// Now we'll calculate the accleration value into actual g's
		ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
		ay = (float)accelCount[1] * aRes;
		az = (float)accelCount[2] * aRes;

		mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
		gRes = mpu.getGres();

		// Calculate the gyro value into actual degrees per second
		gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
		gy = (float)gyroCount[1] * gRes;
		gz = (float)gyroCount[2] * gRes;

		tempCount = mpu.readTempData();  // Read the x/y/z adc values
		temperature = ((float)tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
	}

	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;
	//    if(lastUpdate - firstUpdate > 10000000uL) {
	//      beta = 0.041; // decrease filter gain after stabilized
	//      zeta = 0.015; // increase gyro bias drift gain after stabilized
	//    }
	// Pass gyro rate as rad/s
	MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f);

	// Serial print and/or display at 0.5 s rate independent of data rates
	delt_t = millis() - count;
	if (delt_t > 500) { // update LCD once per half-second independent of read rate
		digitalWrite(blinkPin, blinkOn);

		yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

		pitch *= 180.0f / PI;
		yaw *= 180.0f / PI;
		roll *= 180.0f / PI;

		Serial.print("Yaw, Pitch, Roll: ");
		Serial.print(yaw, 2);
		Serial.print(", ");
		Serial.print(pitch, 2);
		Serial.print(", ");
		Serial.println(roll, 2);

		blinkOn = ~blinkOn;
		count = millis();
	}
}

void setup() {
	delay(5000);
	Wire.begin();
	Serial.begin(9600);

	pinMode(leftTrigPin, OUTPUT);
	pinMode(rightTrigPin, OUTPUT);
	pinMode(leftEchoPin, INPUT);
	pinMode(rightEchoPin, INPUT);
	pinMode(grey_a, INPUT);
	pinMode(grey_b, INPUT);
	pinMode(grey_c, INPUT);
	pinMode(grey_1, INPUT);
	pinMode(grey_2, INPUT);
	pinMode(grey_3, INPUT);
	pinMode(grey_4, INPUT);
	pinMode(grey_5, INPUT);
	pinMode(grey_6, INPUT);
	pinMode(grey_7, INPUT);
	attachInterrupt(5, interrupt, FALLING);		//利用19号口作为中断
//初始化陀螺仪相关内容
	pinMode(intPin, INPUT);
	digitalWrite(intPin, LOW);
	pinMode(blinkPin, OUTPUT);
	digitalWrite(blinkPin, HIGH);
	uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
	if (c == 0x68) // WHO_AM_I should always be 0x68
	{
		Serial.println("MPU6050 is online...");
		mpu.MPU6050SelfTest(SelfTest); 
		if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
			Serial.println("Pass Selftest!");
			mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
			Serial.println("MPU6050 bias");
			Serial.println(" x\t  y\t  z  ");
			Serial.print((int)(1000 * accelBias[0])); Serial.print('\t');
			Serial.print((int)(1000 * accelBias[1])); Serial.print('\t');
			Serial.print((int)(1000 * accelBias[2]));
			Serial.println(" mg");
			Serial.print(gyroBias[0], 1); Serial.print('\t');
			Serial.print(gyroBias[1], 1); Serial.print('\t');
			Serial.print(gyroBias[2], 1);
			Serial.println(" o/s");
			mpu.initMPU6050(); Serial.println("MPU6050 initialized for active data mode....");
		}
		else
		{
			Serial.print("Could not connect to MPU6050: 0x");
			Serial.println(c, HEX);
			while (1); // Loop forever if communication doesn't happen
		}
	}
}

void loop() {
	ypr();
	barrier();
}
