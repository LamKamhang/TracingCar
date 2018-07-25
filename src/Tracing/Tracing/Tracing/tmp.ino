/* gray module */
/* some pins for the gray sensors */
const int _GRAY_IO_PIN_[] = { 1, 2, 3, 4, 5, 6, 7 };

/* some parameter for gray sensors */
const int _GRAY_IO_WEIGHT_[] = { -120, -80, -40, 0, 40, 80, 120 };

int calerror()
{
	int error = 0;
	int sum = 7;
	for (int i = 0; i < 7; ++i)
	{
		// black line --> return 1
		// white --> return 0
		int value = 1 - digitalRead(_GRAY_IO_PIN_[i]);
		sum += value;
		error += value * _GRAY_IO_WEIGHT_[i];
	}

	// if the return value is small, means good.
	// if the return value is positive large, means turn left.
	// if the return value is negative large, means turn right.
	return error / sum;
}

/* PID algorithm's parameters */
#define _KP_	1
#define _KI_	1
#define _KD_	50
#define _DT_	20


int caloutput(int curError)
{
	static int preError = 0;
	static int integral = 0;

	integral += curError * _DT_;
	int diff = (curError - preError) / _DT_;

	int output = _KP_ * curError +
		_KI_ * integral +
		_KD_ * diff;

	preError = curError;

	// adjust two motor's speed.
	// left motor minus it.
	// right motor plus it.
	return output;
}