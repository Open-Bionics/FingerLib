/*
 * sam_FingerTimer.h
 *
 * Created: 02/11/2015 11:12:48
 * Author: Olly McBride
 *
 * This work is licensed under the Creative Commons Attribution 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
 *
 */ 

#if defined(ARDUINO_ARCH_SAMD)

#ifndef SAM_FINGERTIMER_H_
#define SAM_FINGERTIMER_H_

#include <Arduino.h>

// select timer frequency
//#define TIMER_10KHZ
//#define TIMER_5KHZ
#define TIMER_2KHZ
//#define TIMER_1KHZ
//#define TIMER_500HZ


#if defined(TIMER_10KHZ)
#define TIMER_FREQ 10			// timer frequency in KHz
#define CC_REG_VAL 2397			// compare capture reg val
#elif defined(TIMER_5KHZ)
#define TIMER_FREQ 5			// timer frequency in KHz
#define CC_REG_VAL 4794			// compare capture reg val
#elif defined(TIMER_2KHZ)
#define TIMER_FREQ 2			// timer frequency in KHz
#define CC_REG_VAL 11985		// compare capture reg val
#elif defined(TIMER_1KHZ)
#define TIMER_FREQ 1			// timer frequency in KHz
#define CC_REG_VAL 23970		// compare capture reg val
#elif defined(TIMER_500HZ)
#define TIMER_FREQ 0.5			// timer frequency in KHz
#define CC_REG_VAL 47940		// compare capture reg val
#endif

#define ms(val)		((val)*(TIMER_FREQ))		// number of timer ticks per ms

// calculate number of timer ticks per ms for each timer function
#define MILLI_TIME        ms(1)					// 1ms		1Hz	
#define MOTOR_CTRL_TIME   ms(5)					// 5ms		200Hz

// function prototypes
void _timerSetup(void);
void TC4_Handler();
long customMillis(void);

void _passMotorPtr(void (*f)(void));
void _attachFuncToTimer(void (*f)(void));

#endif /* SAM_FINGERTIMER_H_ */
#endif /* defined(ARDUINO_ARCH_SAMD) */