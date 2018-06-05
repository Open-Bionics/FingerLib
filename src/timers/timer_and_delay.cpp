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

////////////////////// NON_BLOCKING DELAY CLASS //////////////////////////

NB_DELAY_CLASS::NB_DELAY_CLASS()
{
	_started = false;
	_startTime = 0;
}

NB_DELAY_CLASS::~NB_DELAY_CLASS()
{

}

// start non-blocking delay
void NB_DELAY_CLASS::start(long delVal)
{
	_startTime = ticker();
	_interval = delVal;
	_started = true;
}

// returns true if _interval has elapsed
bool NB_DELAY_CLASS::finished(void)
{
	if (((ticker() - _startTime) >= _interval) && _started)
	{
		_started = false;
		return true;
	}
	else
	{
		return false;
	}
}

// returns true if timer is running
bool NB_DELAY_CLASS::started(void)
{
	return _started;
}

// returns elapsed time, if not started return 0
long NB_DELAY_CLASS::now(void)
{
	if (!_started)
	{
		return 0;
	}
	else
	{
		return (ticker() - _startTime);
	}
}

// stop the timer and return elapsed time
long NB_DELAY_CLASS::stop(void)
{
	long elapsed = now();

	_started = false;

	return elapsed;
}

// run the timer constantly, return true every time the delVal has elapsed
bool NB_DELAY_CLASS::timeElapsed(long delVal)
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

// return the interval/delVal currently being used
long NB_DELAY_CLASS::getInterval(void)
{
	return _interval;
}

// function used to increment the timer after a particular period (us, ms, seconds etc)
unsigned long MS_NB_DELAY::ticker(void)
{
	return millis();
}

// function used to increment the timer after a particular period (us, ms, seconds etc)
unsigned long US_NB_DELAY::ticker(void)
{
	return micros();
}



////////////////////// NON_BLOCKING TIMER CLASS //////////////////////////

NB_TIMER_CLASS::NB_TIMER_CLASS()
{
	_startTime = 0;
	_started = false;

	_pauseTime = 0;
	_paused = false;
}

NB_TIMER_CLASS::~NB_TIMER_CLASS()
{

}

// start non-blocking timer
void NB_TIMER_CLASS::start(void)
{
	_startTime = ticker();
	_started = true;
	_paused = false;
}

// returns true if delay timer is running	
bool NB_TIMER_CLASS::started(void)
{
	return _started;
}

// returns how much time has elapsed since the start, if not started return 0
long NB_TIMER_CLASS::now(void)
{
	if (!_started)
	{
		return 0;
	}
	else if (_paused)
	{
		return (_pauseTime - _startTime);
	}
	else
	{
		return (ticker() - _startTime);
	}
}

// stop the timer and return elapsed time
long NB_TIMER_CLASS::stop(void)
{
	long elapsed = now();

	_started = false;

	return elapsed;
}

// stop the timer, return the elapsed time and start the timer again 
long NB_TIMER_CLASS::restart(void)
{
	long elapsed = stop();
	start();
	return elapsed;
}

// check whether a certain interval has elapsed, keep timer running
bool NB_TIMER_CLASS::timeElapsed(long interval)
{
	if (now() >= interval)
	{
		return true;
	}
	else
	{
		return false;
	}
}

// pause the timer
long NB_TIMER_CLASS::pause(void)
{
	if (!_started)
	{
		return 0;
	}

	_pauseTime = ticker();
	_paused = true;

	return (_pauseTime - _startTime);
}

// return true of the timer is currently paused
bool NB_TIMER_CLASS::paused()
{
	return _paused;
}

// resume the timer
long NB_TIMER_CLASS::resume(void)
{
	_startTime += (ticker() - _pauseTime);
	_paused = false;

	return now();
}


// function used to increment the timer after a particular period (us, ms, seconds etc)
unsigned long MS_NB_TIMER::ticker(void)
{
	return millis();
}

// function used to increment the timer after a particular period (us, ms, seconds etc)
unsigned long US_NB_TIMER::ticker(void)
{
	return micros();
}