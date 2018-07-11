/*
 * timer_and_delay.h
 *
 * Created: 28/09/2016 10:57:02
 * Author: Olly McBride
 *
 * This work is licensed under the Creative Commons Attribution 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
 *
 */ 


#ifndef TIMER_AND_DELAY_H_
#define TIMER_AND_DELAY_H_

#include <Arduino.h>



 /**
 * # Non-blocking delay base class
 *
 * The timer and delay unit is designed to indicate elapsed duration, and can be used as
 * a non-blocking delay function or to measure the amount of time a process has taken.
 * The timer and delay classes are compatible with both milliseconds and microseconds by
 * creating instances of any of the following;
 *
 * 1. MS_NB_DELAY - millisecond delay
 * 2. US_NB_DELAY - microsecond delay
 * 3. MS_NB_TIMER - millisecond timer
 * 4. US_NB_TIMER - microsecond timer
 *
 * The timer and delay classes use a virtual 'ticker()' function that is used for the
 * duration calculation. This ticker() function is set to return a timestamp in millisecond
 * or microseconds, but can also be spoofed by the unit test process.
 *
 */


class NB_DELAY_CLASS
{
	public:
	NB_DELAY_CLASS();
	~NB_DELAY_CLASS();

	void start(long delVal);		/**< start non-blocking delay */
	bool finished(void);			/**< returns true if delVal has elapsed */
	bool started(void);				/**< returns true if delay timer is running */
	long now(void);					/**< returns elapsed time, if not started return 0 */
	long stop(void);				/**< stop the timer and return elapsed time */

	bool timeElapsed(long delVal);	/**< run the timer constantly, return true every time the delVal has elapsed */

	long getInterval(void);			/**< return the interval/delVal currently being used */

	private:
	long _startTime;				/**< time at which the delay was started */
	long _interval;					/**< amount of time the delay will last for */
	bool _started;					/**< flag to indicate if the delay has started */

	virtual unsigned long ticker(void)	/**< virtual function used to track the change in time (us, ms, seconds etc) */
	{
		return 0;
	}
};

/*! Millisecond non-blocking delay class */
class MS_NB_DELAY : public NB_DELAY_CLASS
{
	public:
	unsigned long ticker(void);
};

/*! Microsecond non-blocking delay class */
class US_NB_DELAY : public NB_DELAY_CLASS
{
	public:
	unsigned long ticker(void);
};


/**
* # Non-blocking timer base class
*
* The timer and delay unit is designed to indicate elapsed duration, and can be used as
* a non-blocking delay function or to measure the amount of time a process has taken.
* The timer and delay classes are compatible with both milliseconds and microseconds by
* creating instances of any of the following;
*
* 1. MS_NB_DELAY - millisecond delay
* 2. US_NB_DELAY - microsecond delay
* 3. MS_NB_TIMER - millisecond timer
* 4. US_NB_TIMER - microsecond timer
*
* The timer and delay classes use a virtual 'ticker()' function that is used for the
* duration calculation. This ticker() function is set to return a timestamp in millisecond
* or microseconds, but can also be spoofed by the unit test process.
*
*/
class NB_TIMER_CLASS
{
	public:
	NB_TIMER_CLASS();
	~NB_TIMER_CLASS();

	void start(void);				/**< start non-blocking timer */
	bool started(void);				/**< returns true if delay timer is running */
	long now(void);					/**< returns how much time has elapsed since the start, if not started return 0 */
	long stop(void);				/**< stop the timer and return elapsed time */
	long restart(void);				/**< stop the timer, return the elapsed time and start the timer again */

	bool timeElapsed(long interval);/**< check whether a certain interval has elapsed, keep timer running */

	long pause(void);				/**< pause the timer */
	bool paused(void);				/**< return true of the timer is currently paused */
	long resume(void);				/**< resume the timer */

	private:
	long _startTime;				/**< time at which the timer was started */
	bool _started = false;			/**< flag to indicate if the timer has started */


	long _pauseTime;				/**< time at which the timer was paused */
	bool _paused;					/**< flag to indicate if the timer is paused */

	virtual unsigned long ticker(void)	/**< function used to increment the timer after a particular period (us, ms, seconds etc) */
	{
		return 0;
	}
};

/*! Millisecond non-blocking timer class */
class MS_NB_TIMER : public NB_TIMER_CLASS
{
	public:
	unsigned long ticker(void);
};

/*! Microsecond non-blocking timer class */
class US_NB_TIMER : public NB_TIMER_CLASS
{
	public:
	unsigned long ticker(void);
};

#endif /* TIMER_AND_DELAY_H_ */