/*
 * sam_FingerTimer.cpp
 *
 * Created: 02/11/2015 11:12:48
 * Author: Olly McBride
 *
 * This work is licensed under the Creative Commons Attribution 4.0 International License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
 *
 */ 


#if defined(ARDUINO_ARCH_SAMD)

#include "samd_FingerTimer.h"

// used for customMillis()
//unsigned long _milliSeconds = 0;

// create global pointers to functions and flags
void (*_ptr2MotorFunc)(void) = NULL;
void (*_ptr2PiggybackFunc)(void) = NULL;

// function to receive pointer to motor and assign to global pointer
void _passMotorPtr(void (*f)(void))
{
	_ptr2MotorFunc = f;
}

// function to receive pointer for timer piggybacking and assign to global pointer
void _attachFuncToTimer(void (*f)(void))
{
	_ptr2PiggybackFunc = f;
}

// initialise timer registers for 5KHz timer (200uS)
void _posCtrlTimerSetup(void)
{
	// Enable clock for TC4 
	REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5) ;
	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync 

	// The type cast must fit with the selected timer mode 
	TcCount16* TC = (TcCount16*) TC4;		// get timer struct

	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;	// Disable TC
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

	TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;	// Set Timer counter Mode to 16 bits
	while (TC->STATUS.bit.SYNCBUSY == 1);		// wait for sync 
	TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;	// Set TC Freq
	while (TC->STATUS.bit.SYNCBUSY == 1);		// wait for sync 

	TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;		// Set prescaler
	while (TC->STATUS.bit.SYNCBUSY == 1);			// wait for sync 

	TC->CC[0].reg = CC_REG_VAL;			// timer frequency
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
  
	// Interrupts 
	TC->INTENSET.reg = 0;              // disable all interrupts
	TC->INTENSET.bit.OVF = 1;          // enable overflow
	TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0

	// Enable InterruptVector
	NVIC_EnableIRQ(TC4_IRQn);

	// Enable TC
	TC->CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  
}

// TC4
void TC4_Handler()			// also used as PWM timer for D16 & D17 (not used on any OB Samd hand) 
{
	static long timerCount = 0;			// main timer counter increments every call of the interrupt
	static long motorCount = 0;			// time instance variable for motor position control
	static long mSecCount = 0;			// time instance variable for millisecond counter
	
	TcCount16* TC = (TcCount16*) TC4;	// get timer struct
	if (TC->INTFLAG.bit.OVF == 1)		// An overflow caused the interrupt
	{  
		TC->INTFLAG.bit.OVF = 1;		// writing a one clears the flag ovf flag

		timerCount++;					// increment timer counter every period
		
		// triggered once a millisecond
		if((timerCount - mSecCount) >= MILLI_TIME)
		{
			mSecCount = timerCount;
			
			if (_ptr2PiggybackFunc)
			{
				_ptr2PiggybackFunc();
			}	
		}
 
		// position control for a single motor
		if((timerCount - motorCount) >= MOTOR_CTRL_TIME)
		{
			motorCount = timerCount;

			if (_ptr2MotorFunc != NULL)
			{
				_ptr2MotorFunc();
			}
		}
	}
  
	if (TC->INTFLAG.bit.MC0 == 1)	// a compare to cc0 caused the interrupt
	{  
		TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
	}
}

#endif /* defined(ARDUINO_ARCH_SAMD) */