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

#if defined(ARDUINO_AVR_MEGA2560)

#include "avr_FingerTimer.h"

// used for customMillis()
unsigned long _milliSeconds = 0;

// create global pointers to functions and flags
void (*_ptr2MotorFunc)(void) = NULL;
void (*_ptr2PiggybackFunc)(void) = NULL;
int _ptr2PiggybackFlag = false;

// function to receive pointer to motor and assign to global pointer
void _passMotorPtr(void (*f)(void))
{
	_ptr2MotorFunc = f;
}

// function to receive pointer for timer piggybacking and assign to global pointer
void _attachFuncToTimer(void (*f)(void))
{
	_ptr2PiggybackFunc = f;
	
	_ptr2PiggybackFlag = true;
}

// initialise timer registers for 5KHz timer (200uS)
void _timerSetup(void)      
{
  _changePWMFreq();  // change PWM to 31KHz, so it is out of the audible range
                  
  cli();	//stop interrupts

  // Timer5 5Khz timer 200uS
  TCCR5A = 0;	// set entire TCCR5A register to 0
  TCCR5B = 0;	// same for TCCR5B
  TCNT5  = 0;	//initialize counter value to 0
  // set compare match register for 5khz increments
  OCR5A = CC_REG_VAL;	// = (16*10^6) / (1*5000) - 1 (must be <65536)
  // turn on CTC mode
  TCCR5B |= (1 << WGM52);
  // Set CS50 bit for no prescaler for maximum precision
  TCCR5B |= (1 << CS50);  
  // enable timer compare interrupt
  TIMSK5 |= (1 << OCIE5A);

  sei();//allow interrupts
}

// Timer5 at 5KHz (200uS)
ISR(TIMER5_COMPA_vect)    
{
  static long timer5cnt = 0;        // main timer counter increments every call of the interrupt
  static long servoCount = 0;     // time instance variable for motor position control
  static long mSecCount = 0;		// time instance variable for millisecond counter

  timer5cnt++;    // increment timer counter every 200uS
  
  // triggered once a millisecond
  if((timer5cnt - mSecCount) >= MILLI_TIME)
  {
	  mSecCount = timer5cnt;
	  _milliSeconds++;
	  
	  if(_ptr2PiggybackFlag)
	  {
		  _ptr2PiggybackFunc();
	  }
  }
  
  // position control for a single motor
  if((timer5cnt - servoCount) >= MOTOR_CTRL_TIME)
  {
    servoCount = timer5cnt;
	_ptr2MotorFunc();
  }
}

void _changePWMFreq(void)
{
	TCCR1B = (TCCR1B & 0b11111000) | 0x01;
	TCCR2B = (TCCR2B & 0b11111000) | 0x01;
	TCCR3B = (TCCR3B & 0b11111000) | 0x01;
	TCCR4B = (TCCR4B & 0b11111000) | 0x01;
}

long customMillis(void)   // similar to Millis(), but Millis() may not function due to using Timer5
{
	return _milliSeconds;
}

#endif /* defined(ARDUINO_AVR_MEGA2560) */