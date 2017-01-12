/*
* samd_CurrentSense.cpp
*
* Created: 16/09/2015 11:30:18
* Author: Olly McBride
*
* This work is licensed under the Creative Commons Attribution 4.0 International License.
* To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
*
*/

#if defined(ARDUINO_ARCH_SAMD)

#include "samd_CurrentSense.h"

// list to hold pointers to each timer instance
const void* TCInstances[NUM_TIMERS] = { TCC0,TCC1,TCC2,TC3,TC4,TC5 };

// list to hold which timers have been enabled
int PWMTimerList[NUM_TIMERS];

// the PWM timer being used to read the current
uint8_t _currentSenseTimer = (uint8_t)(-1);

void(*_ptr2CurrSnsFunc)(void) = NULL;
int _ptr2CurrSnsFuncFlag = false;

// attach current sense control interrupt, initialise and sync PWM timers
void initCurrentSense(uint8_t dir0, uint8_t dir1, void(*f)(void))
{
	static bool _currSnsInit = false;				// flag for current sense initialisation


	// attach current sense function
	if (!_currSnsInit)
	{
		_currSnsInit = true;

		_ptr2CurrSnsFunc = f;
		_ptr2CurrSnsFuncFlag = true;

		setFastAnalogRead();
	}

	// initalise PWM timers
	uint8_t timer0 = initPWMtimer(dir0);
	uint8_t timer1 = initPWMtimer(dir1);

	// increment the list to show which timers are being used for PWM, so they can be synced later
	if (timer0 != -1)
		PWMTimerList[timer0]++;

	if (timer1 != -1)
		PWMTimerList[timer1]++;

	// sync all attached PWM timers
	syncPWMTimers();
}



// change the ADC clock prescaler to DIV (to reduce time in curr sense interrupt)
void setFastAnalogRead(void)
{
	ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV128_Val;					// ollyEdit
	syncADC();					// ollyEdit

}

// initialise the PWM timer with custom freq and prescaler
uint8_t initPWMtimer(uint32_t pin)
{
	PinDescription pinDesc = g_APinDescription[pin];
	uint32_t attr = pinDesc.ulPinAttribute;

	//MYSERIAL.print("Pin: ");
	//MYSERIAL.print(pin);

	// let arduino initialise timers & peripheral pins
	analogWrite(pin, 0);

	// if pin is a PWM pin
	if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
	{
		uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
		uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
		static bool tcEnabled[TCC_INST_NUM + TC_INST_NUM];

		// if timer has not already been configured, config timer
		if (!tcEnabled[tcNum])
		{
			tcEnabled[tcNum] = true;

			// configure peripheral according to device
			if (attr & PIN_ATTR_TIMER)
			{
#if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
				// Compatibility for cores based on SAMD core <=1.6.2
				if (pinDesc.ulPinType == PIO_TIMER_ALT)
				{
					pinPeripheral(pin, PIO_TIMER_ALT);
				}
				else
#endif
				{
					pinPeripheral(pin, PIO_TIMER);
				}
			}
			else
			{
				// We suppose that attr has PIN_ATTR_TIMER_ALT bit set...
				pinPeripheral(pin, PIO_TIMER_ALT);
			}

			// assign timer to GCLK
			uint16_t GCLK_CLKCTRL_IDs[] =
			{
				GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
				GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
				GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
				GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
				GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
				GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
				GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
				GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
			};
			GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
			while (GCLK->STATUS.bit.SYNCBUSY == 1);

			// configure timer for PWM
			if (tcNum < TCC_INST_NUM)			// if TCCx
			{
				// -- Configure TCC
				Tcc* TCCx = (Tcc*)GetTC(pinDesc.ulPWMChannel);
				// Disable TCCx
				TCCx->CTRLA.bit.ENABLE = 0;
				syncTCC(TCCx);
				// Set prescaler to 1/256
				//TCCx->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV256;
				TCCx->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV8;
				//syncTCC(TCCx);
				// Set TCx as normal PWM
				TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
				syncTCC(TCCx);
				// Set the initial value to 0
				TCCx->CC[tcChannel].reg = (uint32_t)0x00;
				syncTCC(TCCx);
				// Set PER to maximum counter value (resolution : 0xFF)
				TCCx->PER.reg = 0xFF;
				syncTCC(TCCx);

				// Interrupts 
				TCCx->INTENSET.reg = 0;              // disable all interrupts
				TCCx->INTENSET.bit.OVF = 1;          // enable overfollow
				TCCx->INTENSET.bit.MC0 = 1;          // enable compare match to CC0

													 // Enable TCCx
				TCCx->CTRLA.bit.ENABLE = 1;
				syncTCC(TCCx);
			}
			else 			// if TCx
			{
				// -- Configure TC
				Tc* TCx = (Tc*)GetTC(pinDesc.ulPWMChannel);
				// Disable TCx
				TCx->COUNT8.CTRLA.bit.ENABLE = 0;
				syncTC_8(TCx);
				// Set Timer counter Mode to 8 bits, normal PWM, prescaler 1/256
				//TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_WAVEGEN_NPWM | TC_CTRLA_PRESCALER_DIV256;
				TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_WAVEGEN_NPWM | TC_CTRLA_PRESCALER_DIV8;
				syncTC_8(TCx);
				// Set the initial value to 0
				TCx->COUNT8.CC[tcChannel].reg = (uint8_t)0x00;
				syncTC_8(TCx);
				// Set PER to maximum counter value (resolution : 0xFF)
				TCx->COUNT8.PER.reg = 0xFF;
				syncTC_8(TCx);
				// Enable TCx
				TCx->COUNT8.CTRLA.bit.ENABLE = 1;
				syncTC_8(TCx);
			}
		}
		else
		{
			if (tcNum >= TCC_INST_NUM)
			{
				Tc* TCx = (Tc*)GetTC(pinDesc.ulPWMChannel);
				TCx->COUNT8.CC[tcChannel].reg = (uint8_t)0x00;
				syncTC_8(TCx);
			}
			else {
				Tcc* TCCx = (Tcc*)GetTC(pinDesc.ulPWMChannel);
				TCCx->CTRLBSET.bit.LUPD = 1;
				syncTCC(TCCx);
				TCCx->CCB[tcChannel].reg = (uint32_t)0x00;
				syncTCC(TCCx);
				TCCx->CTRLBCLR.bit.LUPD = 1;
				syncTCC(TCCx);
			}
		}

		// return the PWM timer used for the pin
		return tcNum;
	}
	else
	{
		// if pin is not a PWM pin, return -1
		return (uint8_t)(-1);
	}
}




// count through every timer, if it is being used for PWM (timerList[i] > 0) disable timer and reset all counters to 0
// when every PWM timer is clear, re-enable all PWM timers and return number of synced PWM timers
uint8_t syncPWMTimers(void)
{
	uint8_t nTimers = 0;

	Tcc* TCC[TC_INST_NUM];
	Tc* TC[TC_INST_NUM];

	// disable all active PWM timers, clear timer counters and prescale counters
	for (int i = 0; i < NUM_TIMERS; i++)
	{
		if (!PWMTimerList[i])				// if timer is not used for PWM, continue
			continue;

		if (i < TCC_INST_NUM)			// if TCCx
		{
			TCC[i] = (Tcc*)TCInstances[i];
			// Disable TCCx
			TCC[i]->CTRLA.bit.ENABLE = 0;
			syncTCC(TCC[i]);

			// Clear the prescaler counter
			TCC[i]->CTRLA.reg |= TCC_CTRLA_PRESCSYNC_RESYNC;
			syncTCC(TCC[i]);

			// Clear the timer counter
			TCC[i]->COUNT.reg = 0x00;
			syncTCC(TCC[i]);

		}
		else							// if TCx
		{
			int j = i - TCC_INST_NUM;
			TC[j] = (Tc*)TCInstances[i];

			// Disable TCCx
			TC[j]->COUNT8.CTRLA.bit.ENABLE = 0;
			syncTC_8(TC[j]);

			// Clear the prescaler counter
			TC[j]->COUNT8.CTRLA.reg |= TC_CTRLA_PRESCSYNC_RESYNC;
			syncTC_8(TC[j]);

			// Clear the timer counter
			TC[j]->COUNT8.COUNT.reg = 0x00;
			syncTC_8(TC[j]);
		}
	}


	// use lowest timer for current sense
	for (int i = 0; i < NUM_TIMERS; i++)
	{
		if (!PWMTimerList[i])				// if timer is not used for PWM, continue
			continue;

		enableCurrentSnsTimer(i);
		break;
	}

	// re-enable all PWM timers
	for (int i = 0; i < NUM_TIMERS; i++)
	{
		if (!PWMTimerList[i])				// if timer is not used for PWM, continue
			continue;

		if (i < TCC_INST_NUM)			// if TCCx
		{
			// Enable TCCx
			TCC[i]->CTRLA.bit.ENABLE = 1;
			syncTCC(TCC[i]);
		}
		else							// if TCx
		{
			int j = i - TCC_INST_NUM;

			// Enable TCx
			TC[j]->COUNT8.CTRLA.bit.ENABLE = 1;
			syncTC_8(TC[j]);
		}

		nTimers++;						// if PWM timers, increment counter to show sync success
	}

	return nTimers;		// return number of PWM timers synced 
}

// enable timer interrupt for current reading
void enableCurrentSnsTimer(uint8_t tNum)
{
	// list of Interrupt Requests for timers
	IRQn_Type timeIRQnList[NUM_TIMERS] =
	{
		TCC0_IRQn,
		TCC1_IRQn,
		TCC2_IRQn,
		TC3_IRQn,
		TC4_IRQn,
		TC5_IRQn
	};

	// enable Interrupt Vector
	NVIC_EnableIRQ(timeIRQnList[tNum]);

	// set sense timer number
	_currentSenseTimer = tNum;
}





// PWM timer interrupt handlers
void TCC0_Handler()
{
	const uint8_t tmrNum = 0;		// TCC0

	Tcc* TCC = (Tcc*)TCC0;			// get timer struct
	if (TCC->INTFLAG.bit.OVF == 1)	// A overflow caused the interrupt
	{

		if (_currentSenseTimer == tmrNum)
		{
			if (_ptr2CurrSnsFuncFlag)
			{
				_ptr2CurrSnsFunc();
			}
		}


		TCC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	}
}

void TCC1_Handler()
{
	const uint8_t tmrNum = 1;		// TCC1

	Tcc* TCC = (Tcc*)TCC1;			// get timer struct
	if (TCC->INTFLAG.bit.OVF == 1)	// A overflow caused the interrupt
	{

		if (_currentSenseTimer == tmrNum)
		{
			if (_ptr2CurrSnsFuncFlag)
			{
				_ptr2CurrSnsFunc();
			}
		}


		TCC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	}
}

void TCC2_Handler()
{
	const uint8_t tmrNum = 2;		// TCC2

	Tcc* TCC = (Tcc*)TCC2;			// get timer struct
	if (TCC->INTFLAG.bit.OVF == 1)	// A overflow caused the interrupt
	{

		if (_currentSenseTimer == tmrNum)
		{
			if (_ptr2CurrSnsFuncFlag)
			{
				_ptr2CurrSnsFunc();
			}
		}


		TCC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	}
}

void TC3_Handler()
{
	const uint8_t tmrNum = 3;		// TC3

	TcCount8* TC = (TcCount8*)TC3;	// get timer struct
	if (TC->INTFLAG.bit.OVF == 1)
	{
		if (_currentSenseTimer == tmrNum)
		{
			if (_ptr2CurrSnsFuncFlag)
			{
				_ptr2CurrSnsFunc();
			}
		}

		TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	}
}

// being used for position control
//void TC4_Handler()
//{
//	const uint8_t tmrNum = 4;		// TC4
//
//	TcCount8* TC = (TcCount8*)TC4;	// get timer struct
//	if (TC->INTFLAG.bit.OVF == 1)
//	{
//		if (_currentSenseTimer == tmrNum)
//		{
//			readCurrSense();
//		}
//
//		TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
//	}
//}

void TC5_Handler()
{
	const uint8_t tmrNum = 5;		// TC5

	TcCount8* TC = (TcCount8*)TC5;	// get timer struct
	if (TC->INTFLAG.bit.OVF == 1)
	{
		if (_currentSenseTimer == tmrNum)
		{
			if (_ptr2CurrSnsFuncFlag)
			{
				_ptr2CurrSnsFunc();
			}
		}

		TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	}
}


#endif /* defined(ARDUINO_ARCH_SAMD) */