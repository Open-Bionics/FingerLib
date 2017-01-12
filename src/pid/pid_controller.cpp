/*
* pid_controller.cpp
*
* Created: 29/09/2016 13:48:12
* Author: Olly McBride
*
* This work is licensed under the Creative Commons Attribution 4.0 International License.
* To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
*
*/

#include "pid_controller.h"


//#define MYSERIAL SerialUSB

PID_CONTROLLER::PID_CONTROLLER()
{
	_integral = 0;		// clear PID history

	setLimits(DEFAULT_LIMIT_MIN, DEFAULT_LIMIT_MAX);
	setGains(DEFAULT_GAIN_P, DEFAULT_GAIN_I, DEFAULT_GAIN_D);

	enable();
}

// enable the PID controller (default)
void PID_CONTROLLER::enable(void)
{
	_en = true;
}

// disable the PID controller
void PID_CONTROLLER::disable(void)
{
	_en = false;
}

static int PIDnum = 0;


double PID_CONTROLLER::run(double targ, double curr)
{
	if (!_en)
		return 0;

	double output = 0;
	double error = targ - curr;
	double sampleTime = _sampleTimer.stop() / 1000;		// sampleTime in ms

	if (sampleTime > 0)
	{
		// integral summation allows _Ki to change without causing unwanted disturbances 
		_integral += _Ki * error * sampleTime;
		_integral = constrain(_integral, _min, _max);

		// store the change in input to remove the derivative spike
		double _dInput = (curr - _prev) / sampleTime;

		// calculate output
		output = (_Kp * error) + _integral - (_Kd * _dInput);

		output = constrain(output, _min, _max);



		//if (PIDnum == 2)
		//{
		//	#define MYSERIAL SerialUSB
		//	MYSERIAL.print(targ);
		//	MYSERIAL.print(",");
		//	MYSERIAL.print(curr);
		//	MYSERIAL.print(",");
		//	MYSERIAL.println(output);
		//}

		//PIDnum++;
		//if(PIDnum > 3)
		//	PIDnum = 0;

		//// be aware that this will print the values for every finger that is attached, and can result in odd values 
		//#define MYSERIAL SerialUSB
		//MYSERIAL.print(targ);
		//MYSERIAL.print(",");
		//MYSERIAL.print(curr);
		//MYSERIAL.print(",");
		//MYSERIAL.print(error);
		//MYSERIAL.print(",");
		//MYSERIAL.println(output);

		//#define MYSERIAL Serial
		//	static int i = 0;
		//	if (i == 0)
		//	{
		//		MYSERIAL.print(_min);
		//		MYSERIAL.print(",");
		//		MYSERIAL.print(_max);
		//		MYSERIAL.print(",");
		//		MYSERIAL.println(output);
		//	}
		//
		//	i++;
		//	if (i >= 5)
		//		i = 0;


		// store history
		_prev = curr;
	}

	_sampleTimer.start();

	return output;
}

// set the output value limits
void PID_CONTROLLER::setLimits(double min, double max)
{
	_min = min;
	_max = max;
}

// set the controller gains
void PID_CONTROLLER::setGains(float Kp, float Ki, float Kd)
{
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
}

// get the gains of the controller
void PID_CONTROLLER::getGains(float *Kp, float *Ki, float *Kd)
{
	*Kp = _Kp;
	*Ki = _Ki;
	*Kd = _Kd;
}