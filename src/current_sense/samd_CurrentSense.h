/*
 * samd_CurrentSense.h
 *
 * Created: 16/09/2015 11:30:18
 * Author: Olly McBride
 *
 * This work is licensed under the Creative Commons Attribution 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
 *
 */ 

#if defined(ARDUINO_ARCH_SAMD)

#ifndef SAMD_CURRENTSENSE_H_
#define SAMD_CURRENTSENSE_H_

#include <Arduino.h>
#include <wiring_private.h>			// for pinPeripheral

#include "FingerLib.h"				// for MAX_FINGERS


#define NUM_TIMERS	(TCC_INST_NUM + TC_INST_NUM)

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_8(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_8(Tc* TCx)
{
	while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx)
{
	while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC()
{
	while (ADC->STATUS.bit.SYNCBUSY == 1)
		;
}



// attach current sense control interrupt, initialise and sync PWM timers
void initCurrentSense(uint8_t dir0, uint8_t dir1, void(*f)(void));

// initialise the PWM timer with custom freq and prescaler
uint8_t initPWMtimer(uint32_t pin);

// count through every timer, if it is being used for PWM (timerList[i] > 0) disable timer and reset all counters to 0
// when every PWM timer is clear, re-enable all PWM timers and return number of synced PWM timers
uint8_t syncPWMTimers(void);

void enableCurrentSnsTimer(uint8_t tNum);		// enable timer interrupt for current reading
void setFastAnalogRead(void);					// change the ADC clock prescaler to DIV (to reduce time in curr sense interrupt)
												


// list to hold pointers to each timer instance
extern const void* TCInstances[NUM_TIMERS];

// list to hold which timers have been enabled
extern int PWMTimerList[NUM_TIMERS];


#endif /* SAMD_CURRENTSENSE_H_ */
#endif /* defined(ARDUINO_ARCH_SAMD) */