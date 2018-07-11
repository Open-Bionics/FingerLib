/*
 * avr_FingerTimer.cpp
 *
 * Created: 02/11/2015 11:12:48
 * Author: Olly McBride
 *
 * This work is licensed under the Creative Commons Attribution 4.0 International License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
 *
 */ 

#if defined(ARDUINO_ARCH_AVR)

#include "avr_FingerTimer.h"

// create global pointers to functions and flags
void(*_ptr2MotorFunc)(void) = NULL;
void(*_ptr2PiggybackFunc)(void) = NULL;

// function to receive pointer to motor and assign to global pointer
void _passMotorPtr(void(*f)(void))
{
	_ptr2MotorFunc = f;
}

// function to receive pointer for timer piggybacking and assign to global pointer
void _attachFuncToTimer(void(*f)(void))
{
	_ptr2PiggybackFunc = f;
}

#if defined(ARDUINO_AVR_MEGA2560)

// initialise timer registers for position control timer
void _posCtrlTimerSetup(void)
{
	_changePWMFreq();			// change PWM to ~31KHz, so it is out of the audible range

	cli();					//stop interrupts

	// Timer5
	TCCR5A = 0;				// set entire TCCR5A register to 0
	TCCR5B = 0;				// same for TCCR5B
	TCNT5 = 0;				//initialize counter value to 0 
	OCR5A = CC_REG_VAL;		// set compare match register for 5khz increments
	TCCR5B |= (1 << WGM52);	// turn on CTC mode
	TCCR5B |= (1 << CS50);	// Set CS50 bit for no prescaler for maximum precision
	TIMSK5 |= (1 << OCIE5A);	// enable timer compare interrupt

	sei();					//allow interrupts
}

// Timer5 
ISR(TIMER5_COMPA_vect)
{
	static long timer5cnt = 0;	// main timer counter increments every call of the interrupt
	static long motorCount = 0;	// time instance variable for motor position control
	static long mSecCount = 0;	// time instance variable for millisecond counter

	timer5cnt++;    // increment timer counter every

	// triggered once a millisecond
	if ((timer5cnt - mSecCount) >= MILLI_TIME)
	{
		mSecCount = timer5cnt;

		if (_ptr2PiggybackFunc)
		{
			_ptr2PiggybackFunc();
		}
	}

	// position control for a single motor
	if ((timer5cnt - motorCount) >= MOTOR_CTRL_TIME)
	{
		motorCount = timer5cnt;
		
		if (_ptr2MotorFunc)
		{
			_ptr2MotorFunc();
		}
	}
}

void _changePWMFreq(void)
{
	TCCR1B = (TCCR1B & 0b11111000) | 0x01;
	TCCR2B = (TCCR2B & 0b11111000) | 0x01;
	TCCR3B = (TCCR3B & 0b11111000) | 0x01;
	TCCR4B = (TCCR4B & 0b11111000) | 0x01;
}

#elif defined(ARDUINO_AVR_UNO)

// initialise timer registers for position control timer
void _posCtrlTimerSetup(void)
{

	cli();					//stop interrupts

							// Timer1
	TCCR1A = 0;				// set entire TCCR1A register to 0
	TCCR1B = 0;				// same for TCCR1B
	TCNT1 = 0;				//initialize counter value to 0
	OCR1A = CC_REG_VAL;		// set compare match register to the value set by avr_FingerTimer.h	
	TCCR1B |= (1 << WGM12);	// turn on CTC mode	
	TCCR1B |= (1 << CS10);	// Set CS50 bit for no prescaler for maximum precision	
	TIMSK1 |= (1 << OCIE1A);// enable timer compare interrupt

	sei();					//allow interrupts
}

// Timer1
ISR(TIMER1_COMPA_vect)
{
	static long timer1cnt = 0;      // main timer counter increments every call of the interrupt
	static long motorCount = 0;     // time instance variable for motor position control
	static long mSecCount = 0;		// time instance variable for millisecond counter

	timer1cnt++;    // increment timer counter

					// triggered once a millisecond
	if ((timer1cnt - mSecCount) >= MILLI_TIME)
	{
		mSecCount = timer1cnt;

		if (_ptr2PiggybackFunc)
		{
			_ptr2PiggybackFunc();
		}
	}

	// position control for a single motor
	if ((timer1cnt - motorCount) >= MOTOR_CTRL_TIME)
	{
		motorCount = timer1cnt;
		_ptr2MotorFunc();
	}
}

#endif

#endif /* defined(ARDUINO_ARCH_AVR) */