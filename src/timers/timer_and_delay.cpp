/*
* timer_and_delay.cpp
*
* Created: 28/09/2016 10:57:02
* Author: Olly McBride
*
* This work is licensed under the Creative Commons Attribution 4.0 International License.
* To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
*
*/

#include "timer_and_delay.h"

////////////////////// NON_BLOCKING MICRO SECOND DELAY CLASS //////////////////////////
US_NB_DELAY::US_NB_DELAY()
{

}

void US_NB_DELAY::start(long delVal)
{
	_startTime = micros();
	_interval = delVal;
	_started = true;	
}

bool US_NB_DELAY::finished(void)
{
	long now = micros();

	if (((now - _startTime) >= _interval) && _started)
	{
		_started = false;
		return true;
	}
	else
	{
		return false;
	}
}

bool US_NB_DELAY::started(void)
{
	return _started;
}

long US_NB_DELAY::now(void)
{
	if (!_started)
		return (-1);
	else
		return (micros() - _startTime);
}

void US_NB_DELAY::stop(void)
{
	_started = false;
}

bool US_NB_DELAY::timeEllapsed(long delVal)
{
	if (!started())			// if timer is not started (first run of timer)
	{
		start(delVal);		// start timer
		return false;		// duration has not elapsed, so return false
	}
	if (finished())			// if timer is finished
	{
		start(delVal);		// restart timer
		return true;		// duration has elapsed, so restart timer and return true
	}
	else					// else if timer has started but not finished
	{
		return false;		// duration has not elapsed, so return false
	}
}

////////////////////// NON_BLOCKING MICRO SECOND TIMER CLASS //////////////////////////
US_NB_TIMER::US_NB_TIMER()
{

}

void US_NB_TIMER::start(void)
{
	_startTime = micros();
	_started = true;
}

bool US_NB_TIMER::started(void)
{
	return _started;
}

long US_NB_TIMER::now(void)
{
	if (!_started)
		return 0;
	else
		return (micros() - _startTime);
}

long US_NB_TIMER::stop(void)
{
	long timeEllapsed;

	timeEllapsed = now();
	_started = false;

	return timeEllapsed;
}

bool US_NB_TIMER::timeEllapsed(long interval)
{
	if (now() >= interval)
		return true;
	else
		return false;
}






////////////////////// NON_BLOCKING MILLI SECOND DELAY CLASS //////////////////////////
MS_NB_DELAY::MS_NB_DELAY()
{

}

void MS_NB_DELAY::start(long delVal)
{
	_startTime = millis();
	_interval = delVal;
	_started = true;
}

bool MS_NB_DELAY::finished(void)
{
	long now = millis();

	if (((now - _startTime) >= _interval) && _started)
	{
		_started = false;
		return true;
	}
	else
	{
		return false;
	}
}

bool MS_NB_DELAY::started(void)
{
	return _started;
}

long MS_NB_DELAY::now(void)
{
	if (!_started)
		return (-1);
	else
		return (millis() - _startTime);
}

void MS_NB_DELAY::stop(void)
{
	_started = false;
}

bool MS_NB_DELAY::timeEllapsed(long delVal)
{
	if (!started())			// if timer is not started (first run of timer)
	{
		start(delVal);		// start timer
		return false;		// duration has not elapsed, so return false
	}
	if (finished())			// if timer is finished
	{
		start(delVal);		// restart timer
		return true;		// duration has elapsed, so restart timer and return true
	}
	else					// else if timer has started but not finished
	{
		return false;		// duration has not elapsed, so return false
	}
}

////////////////////// NON_BLOCKING MILLI SECOND TIMER CLASS //////////////////////////
MS_NB_TIMER::MS_NB_TIMER()
{

}

void MS_NB_TIMER::start(void)
{
	_startTime = millis();
	_started = true;
}

bool MS_NB_TIMER::started(void)
{
	return _started;
}

long MS_NB_TIMER::now(void)
{
	if (!_started)
		return 0;
	else
		return (millis() - _startTime);
}

long MS_NB_TIMER::stop(void)
{
	long timeEllapsed;

	timeEllapsed = now();
	_started = false;

	return timeEllapsed;
}

bool MS_NB_TIMER::timeEllapsed(long interval)
{
	if (now() >= interval)
		return true;
	else
		return false;
}