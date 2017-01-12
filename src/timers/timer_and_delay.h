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


// NON_BLOCKING MICRO SECOND DELAY CLASS
class US_NB_DELAY
{
public:
	US_NB_DELAY();
	void start(long delVal);
	bool finished(void);
	bool started(void);
	long now(void);
	void stop(void);

	bool timeEllapsed(long delVal);
private:
	long _startTime;
	long _interval;
	bool _started = false;
};

// NON_BLOCKING MICRO SECOND TIMER CLASS
class US_NB_TIMER
{
public:
	US_NB_TIMER();
	void start(void);
	bool started(void);
	long now(void);
	long stop(void);

	bool timeEllapsed(long interval);
private:
	long _startTime;
	bool _started = false;

};


#endif /* TIMER_AND_DELAY_H_ */