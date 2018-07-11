/*
* pid_controller.h
*
* Created: 29/09/2016 13:48:12
* Author: Olly McBride
*
* This work is licensed under the Creative Commons Attribution 4.0 International License.
* To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
*
*/


#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <Arduino.h>
#include "../timers/timer_and_delay.h"


#define DEFAULT_LIMIT_MAX	255
#define DEFAULT_LIMIT_MIN	-255

//#define DEFAULT_GAIN_P		1.9
//#define DEFAULT_GAIN_P		1.2
//#define DEFAULT_GAIN_I		0
//#define DEFAULT_GAIN_D		0

//#define DEFAULT_GAIN_P		5
//#define DEFAULT_GAIN_I		0.001
//#define DEFAULT_GAIN_D		0

//#define DEFAULT_GAIN_P		3
//#define DEFAULT_GAIN_I		0.0013
//#define DEFAULT_GAIN_D		0.1

//#define DEFAULT_GAIN_P		2.5
//#define DEFAULT_GAIN_I		0.0003
//#define DEFAULT_GAIN_D		0.1

#define DEFAULT_GAIN_P		2
#define DEFAULT_GAIN_I		0
#define DEFAULT_GAIN_D		10



class PID_CONTROLLER
{
	public:
		PID_CONTROLLER();

		void enable(void);								// enable the PID controller (default)
		void disable(void);								// disable the PID controller
		void enable(bool en);							// set the PID controller to be enabled/disabled
		void reset(void);

		double run(double targ, double curr);			// run the PID computation

		void setLimits(double min, double max);			// set the output value limits					
		void getLimits(double *min, double *max);		// get the output value limits
		void setRampRate(double ramp);					// set the maximum change in output limit

		void setGains(float Kp, float Ki, float Kd);	// set the controller gains
		void getGains(float *Kp, float *Ki, float *Kd);	// get the controller gains

	private:
		bool _en;										// enable/disable flag

		US_NB_TIMER _sampleTimer;						// duration of sample in us

		double _min, _max;								// output limits
		double _rampLim;								// ramp rate limit

		double _prevOutput;								// previous output (used 

		float _Kp, _Ki, _Kd;							// PID controller gains

		double _dInput;									// change in input over time 
		double _prev;									// previous input

		double _integral;								// integral value

};



#endif /* PID_CONTROLLER_H_ */