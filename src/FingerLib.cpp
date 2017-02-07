/*
 * FingerLib.cpp
 *
 * Created: 02/11/2015 11:12:48
 * Author: Olly McBride
 *
 * This work is licensed under the Creative Commons Attribution 4.0 International License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
 *
 */ 

#include "FingerLib.h"

uint8_t _TotalFingerCount = 0;					// the total number of finger instances
uint8_t _TotalAttachedFingers = 0;				// the total number of attached/configured fingers


Finger* fingerISRList[MAX_FINGERS] = { NULL };	// pointer to an instance of the Finger class
bool _posCtrlTimerInit = false;					// flag to prevent multiple timer initialisations



////////////////////////////// Constructor/Destructor //////////////////////////////
Finger::Finger()
{
	if (fingerIndex >= MAX_FINGERS)				// if too many fingers have been initialised
	{
		MYSERIAL.print("ERROR - Too many fingers attached (");
		MYSERIAL.print(fingerIndex);
		MYSERIAL.println(")");

		fingerIndex = -1;						// set current finger number to be empty
		_isActive = false;						// set current finger as inactive 
	}
	else
	{
		fingerIndex = _TotalFingerCount++;		// count the total number of fingers initialised 
	}


#ifdef FORCE_SENSE			// if force sense is available on the current board, initialise the values
	_targForceDir = 0;		
	_forceLimitFlag = 0;
	_targForce = 0;
	_currForce = 0;			// force values converted to ADC values for quicker maths
	_maxForce = 1023;		// force values converted to ADC values for quicker maths
	_minForce = 0;			// force values converted to ADC values for quicker maths
#endif

}

////////////////////////////// Public Methods //////////////////////////////


// INITIALISATION

// attach pins to a finger using only position control
uint8_t Finger::attach(uint8_t dir0, uint8_t dir1, uint8_t posSns)
{
	return attach(dir0, dir1, posSns, (uint8_t)-1, 0);			// no force sense, do not invert
}

// attach pins to a finger using only position control, but allow the direction to be inverted
uint8_t Finger::attach(uint8_t dir0, uint8_t dir1, uint8_t posSns, bool inv)
{
	return attach(dir0, dir1, posSns, (uint8_t)-1, inv);		// no force sense
}

// attach pins to a finger using position control and force control, and allow the direction to be inverted
uint8_t Finger::attach(uint8_t dir0, uint8_t dir1, uint8_t posSns, uint8_t forceSns, bool inv)
{
	// if the current finger number is valid
	if ((fingerIndex >= 0) && (fingerIndex < MAX_FINGERS))
	{
		// configure pins
		pinMode(dir0, OUTPUT);		// set direction1 pin to output
		pinMode(dir1, OUTPUT);		// set direction2 pin to output
		pinMode(posSns, INPUT);		// set position sense pin to input
#ifdef FORCE_SENSE
		pinMode(forceSns, INPUT);	// set force sense pin to input
#endif

		// attach all finger pins
		_Pin.dir[0] = dir0;
		_Pin.dir[1] = dir1;
		_Pin.posSns = posSns;
#ifdef FORCE_SENSE
		_Pin.forceSns = forceSns;
#endif

		// set limits and initial values
		setPosLimits(MIN_FINGER_POS, MAX_FINGER_POS);
		setSpeedLimits(MIN_FINGER_SPEED, MAX_FINGER_SPEED);
		writeSpeed(MAX_FINGER_SPEED);
		writePos(MIN_FINGER_POS);

		// enable the motor and disable finger inversion 
		_invert = inv;					// store whether to invert the finger direction
		_motorEn = true;				// enable the motor
		_currDir = OPEN;				// set dir to OPEN after initial writePos to configure finger dir

		// if finger is being attached for the first time
		if (!_isActive)
			_TotalAttachedFingers++;	// update the count of total attached fingers

		// add a pointer to the current finger instance to the list for position control calls, used by the ISR
		fingerISRList[fingerIndex] = this;

		// initialise the position control timer
		if (!_posCtrlTimerInit)
		{
			_passMotorPtr(&_fingerControlInterrupt);	// attach the finger control function to the timer
			_posCtrlTimerSetup();						// initialise and start the timer
			_posCtrlTimerInit = true;					
		}

#ifdef FORCE_SENSE
		// attach current sense control interrupt, initialise and sync PWM timers
		initCurrentSense(_Pin.dir[0], _Pin.dir[1], &_currentSenseInterrupt);
#endif

#ifdef ARDUINO_AVR_MEGA2560
		// if using the Atmega2560, set the PWM freq to > 20kHz to prevent humming
		setPWMFreq(dir0, 0x01);		// set PWM frequency to max freq
		setPWMFreq(dir1, 0x01);		// set PWM frequency to max freq		
#endif

		_isActive = true;				// set the current finger to be active
		return fingerIndex;				// return the current finger number
	}
	else								// if the current finger number isn't valid
	{
		MYSERIAL.println("ERROR - Too many fingers attached");
		_isActive = false;				// set the current finger to be inactive 
		return (uint8_t)(-1);			// return BLANK
	}
}

// deactivate the finger
void Finger::detach(void)
{
	_isActive = false;
	_TotalAttachedFingers--;
}

// return true if the current finger is attached and initialised correctly 
bool Finger::attached(void)
{
	return _isActive;
}

// invert the current finger direction
void Finger::invertFingerDir(void)
{
	_invert = !_invert;
}


// LIMITS

// set the maximum and minimum position limits
void Finger::setPosLimits(int min, int max)
{
	// set limits
	_minPos = min;
	_maxPos = max;
}

// set the maximum and minimum speed limits
void Finger::setSpeedLimits(int min, int max)
{
	// set limits
	_minSpeed = min;
	_maxSpeed = max;

#ifdef USE_PID
	_PID.setLimits(-(double)_maxSpeed, _maxSpeed);
#endif
}

#ifdef FORCE_SENSE
// set the maximum and minimum force limits
void Finger::setForceLimits(int min, int max)
{
	// set limits
	_minForce = convertForceToADC(min);				// force values converted to ADC values for quicker maths
	_maxForce = convertForceToADC(max);				// force values converted to ADC values for quicker maths
}
#endif


// POS

// write a target position to the finger
void Finger::writePos(int value)
{
	// constrain position value to limits
	_targPos = constrain((uint16_t)value, _minPos, _maxPos);

	// calculate new position error (to remove false positives in reachedPos() )
	_currErr = (signed int)(_targPos - _currPos);

	//determine direction of travel
	if ((uint16_t)value > _currPos)
		_currDir = CLOSE;
	else
		_currDir = OPEN;
}

// write a change in position to the finger
void Finger::movePos(int value)
{
	// change position
	_targPos += value;

	// constrain position value to limits
	_targPos = constrain((uint16_t)_targPos, _minPos, _maxPos);

	// calculate new position error (to remove false positives in reachedPos() )
	_currErr = (signed int)(_targPos - _currPos);
}

// return the current position
int16_t Finger::readPos(void)
{
	return _currPos;
}

// return the error between the current position and the target position
int16_t Finger::readPosError(void)
{
	return _currErr;
}

// return the target position
uint16_t Finger::readTargetPos(void)
{
	return _targPos;
}

// returns true if position reached
bool Finger::reachedPos(void)
{
	// if the position error is within a default tolerance
	if (abs(readPosError()) < POS_REACHED_TOLERANCE)
		return true;
	else
		return false;
}

// returns true if position reached
bool Finger::reachedPos(uint16_t posErr)
{
	// if the position error is within a tolerance
	if (abs(readPosError()) < (int16_t)posErr)
		return true;
	else
		return false;
}


// DIR

// write a target direction to the finger
void Finger::writeDir(int value)
{
	// store direction
	_currDir = value;

	// set new target position based on input direction
	if (_currDir == OPEN)
		_targPos = _minPos;
	else if (_currDir == CLOSE)
		_targPos = _maxPos;
}

// return the current direction
uint8_t Finger::readDir(void)
{
	return _currDir;
}

// open the finger
void Finger::open(void)
{
	writePos(_minPos);
}

// close the finger
void Finger::close(void)
{
	writePos(_maxPos);
}

// toggle finger between open/closed
void Finger::open_close(void)
{
	open_close(!readDir());
}

// set finger to open/close 
void Finger::open_close(boolean dir)
{
	if (dir == OPEN)
		open();
	else if (dir == CLOSE)
		close();
}


// SPEED

// write a target speed to the finger
void Finger::writeSpeed(int value)
{
	_targSpeed = constrain((uint16_t)value, _minSpeed, _maxSpeed);

#ifdef USE_PID
	_PID.setLimits(-(double)_targSpeed, _targSpeed);
#endif
}

// return the current speed being written to the finger
uint8_t Finger::readSpeed(void)
{
	return _currSpeed;
}

// return the target speed
uint8_t Finger::readTargetSpeed(void)
{
	return _targSpeed;
}


#ifdef FORCE_SENSE
// FORCE

// write a target force in a particular direction (0.0 - 54.95N, OPEN - CLOSE)
void Finger::writeForce(float value, int dir)
{
	_targForce = convertForceToADC(value);		// convert force value (float) to ADC value (uint16_t) for quicker maths
	_targForceDir = dir;						// store the target force direction
	_forceLimitFlag = false;					// clear limit flag when a new limit is set
}

// return the current force value. If force sense is disabled, return blank (-1)
float Finger::readForce(void)
{
	// if force sense is not enabled, return
	if (!_forceSenseEn)
		return (-1);
	else
		return convertADCToForce(readCurrent());
}

// return the latest force sense ADC value
uint16_t Finger::readCurrent(void)
{
	// if force sense is not enabled, return
	if (!_forceSenseEn)
		return 0;
	else
		return _currForce;
}

// return true if the force limit has been reached
bool Finger::reachedForceLimit(void)
{
	return _forceLimitFlag;
}

// read the current force and discount the current spike
void Finger::readCurrentSns(void)
{
	// if still within current spike period
	if (currentSpikeTimer.started() && !currentSpikeTimer.finished())
		_currForce = 0;
	else
		_currForce = analogRead(_Pin.forceSns);
}

// convert ADC current sense value to force value (float)
float Finger::convertADCToForce(int ADCVal)
{
	const float m = CURRENT_SENSE_CONST_M;			// generated from the equation of the line from the force-current graph
	const float c = CURRENT_SENSE_CONST_C;			// generated from the equation of the line from the force-current graph
	const float k = ADC_VALS_PER_MA_DRAW;			// number of ADC values per mA draw of the motor

	float force = (((float)ADCVal / k) - c) / m;	// convert ADC value to current draw to output force
	force = constrain(force, 0, 1000);				// constrain the force value to reasonable limits

	return  force;
}

// convert force value to ADC current sense value (int)
int Finger::convertForceToADC(float force)
{
	const float m = CURRENT_SENSE_CONST_M;			// generated from the equation of the line from the force-current graph
	const float c = CURRENT_SENSE_CONST_C;			// generated from the equation of the line from the force-current graph
	const float k = ADC_VALS_PER_MA_DRAW;			// number of ADC values per mA draw of the motor

	int ADCvals = (int)(((m * force) + c) * k);		// convert the force to current draw to ADC values
	ADCvals = constrain(ADCvals, 0, 1024);			// constrain the ADC value to 10-bit ADC limit

	return ADCvals;
}

#endif	// FORCE_SENSE


// STOP/START

// stop the motor and hold position
void Finger::stopMotor(void)
{
	// set target position to current position
	writePos(readPos());
}

// disable the motor by setting the speed to 0
void Finger::disableMotor(void)
{
	_motorEn = false;
}

// re-enable the motor
void Finger::enableMotor(void)
{
	_motorEn = true;
}

// set motor to be enabled/disabled
void Finger::motorEnable(bool motorEn)
{
	_motorEn = motorEn;
}

#ifdef FORCE_SENSE
// enable force sensing
void Finger::enableForceSense(void)
{
	_forceSenseEn = true;
}

// disable force sensing
void Finger::disableForceSense(void)
{
	_forceSenseEn = false;;
}
#endif

// PRINT

// print the current position (no new line)
void Finger::printPos(void)
{
	printPos(0);
}

// print the current position (new line)
void Finger::printPos(bool newL)
{
	MYSERIAL.print("Pos ");
	MYSERIAL.print(readPos());
	MYSERIAL.print("  ");
	if (newL)
		MYSERIAL.print("\n");
}

// print the current position error (no new line)
void Finger::printPosError(void)
{
	printPosError(0);
}

// print the current position error (new line)
void Finger::printPosError(bool newL)
{
	MYSERIAL.print("Err ");
	MYSERIAL.print(readPosError());
	MYSERIAL.print("  ");
	if (newL)
		MYSERIAL.print("\n");
}

// print the current direction (no new line)
void Finger::printDir(void)
{
	printDir(0);
}

// print the current direction (new line)
void Finger::printDir(bool newL)
{
	const char* _dirString[2] = { "OPEN","CLOSE" };	// direction string

	MYSERIAL.print("Dir ");
	MYSERIAL.print(_dirString[readDir()]);
	MYSERIAL.print("  ");
	if (newL)
		MYSERIAL.print("\n");
}

// print whether the target position has been reached (no new line)
void Finger::printReached(void)
{
	printReached(0);
}

// print whether the target position has been reached (new line)
void Finger::printReached(bool newL)
{
	MYSERIAL.print("Reached ");
	MYSERIAL.print(reachedPos());
	MYSERIAL.print("  ");
	if (newL)
		MYSERIAL.print("\n");
}

// print the current speed (no new line)
void Finger::printSpeed(void)
{
	printSpeed(0);
}

// print the current speed (new line)
void Finger::printSpeed(bool newL)
{
	MYSERIAL.print("Speed ");
	MYSERIAL.print(readSpeed());
	MYSERIAL.print("  ");
	if (newL)
		MYSERIAL.print("\n");
}

// print current position, direction, speed and whether the target position has been reached
void Finger::printDetails(void)
{
	MYSERIAL.print("Finger ");
	MYSERIAL.print(fingerIndex);
	MYSERIAL.print("  ");
	printPos();
	printDir();
	printSpeed();
	printReached(true);		// print new line after

}

// print finger number, pins and limits
void Finger::printConfig(void)
{
	MYSERIAL.print("Finger");
	MYSERIAL.print(fingerIndex);
	MYSERIAL.print(" -");
	MYSERIAL.print(" \tdir0: ");
	MYSERIAL.print(_Pin.dir[0]);
	MYSERIAL.print(" \tdir1: ");
	MYSERIAL.print(_Pin.dir[1]);
	MYSERIAL.print(" \tposSense: ");
	MYSERIAL.print(_Pin.posSns);
#ifdef FORCE_SENSE
	MYSERIAL.print(" \tforceSense: ");
	MYSERIAL.print(_Pin.forceSns);
#endif
	MYSERIAL.print("\tinvert: ");
	MYSERIAL.println(_invert);

	MYSERIAL.print("MinPos: ");
	MYSERIAL.print(_minPos);
	MYSERIAL.print("\tMaxPos: ");
	MYSERIAL.println(_maxPos);

	MYSERIAL.print("MinSpeed: ");
	MYSERIAL.print(_minSpeed);
	MYSERIAL.print("\tMaxSpeed: ");
	MYSERIAL.println(_maxSpeed);

#ifdef FORCE_SENSE
	MYSERIAL.print("MinForce: ");
	MYSERIAL.print(_minForce);
	MYSERIAL.print("\tMaxForce: ");
	MYSERIAL.println(_maxForce);
#endif


	MYSERIAL.print("\n");
}


//
void Finger::control(void)
{
	// read finger position
	_currPos = analogRead(_Pin.posSns);

	// invert finger direction if enabled
	if (_invert)
		_currPos = 1023 - _currPos;

#ifdef FORCE_SENSE
	// if force sense is enabled, run force control
	if (_forceSenseEn)
		forceController();

#endif

	positionController();	// run the position controller 
}








////////////////////////////// Private Methods //////////////////////////////

#if defined(USE_PID)
// position controller (using PID)
void Finger::positionController(void)
{
	int16_t speed = 0;

	//// read finger position
	//_currPos = analogRead(_Pin.posSns);

	// invert finger direction if enabled
	if (_invert)
		_currPos = 1023 - _currPos;

	// if motor enabled
	if (_motorEn)
		speed = _PID.run(_targPos, _currPos);

	motorControl(speed);
}

#else
// position controller (using custom P controller)
// controls motor PWM values based on current and target position using a proportional controller (triggered by interrupt)
// total duration = 439us, therefore max freq = 2kHz. We use 200Hz (5ms), where 0.5ms = motor control, 4.5ms = program runtime
void Finger::positionController(void)
{
	signed int motorSpeed = 0;			// used to calculate the motor speed as a vector (±255)
	float m;							// the proportional gradient 
	signed int vectorise = 1;			// changes the sign '±' of the value

#if defined(ARDUINO_AVR_MEGA2560)
	//int proportionalOffset = 150;
	//signed int motorStopOffset = 20;

	int proportionalOffset = 300;
	signed int motorStopOffset = 25;
#elif defined(ARDUINO_SAMD_ZERO)
	int proportionalOffset = 300;
	signed int motorStopOffset = 2;

	//int proportionalOffset = 390;
	//signed int motorStopOffset = 25;
#endif

	//// read position
	//_currPos = analogRead(_Pin.posSns);			// 424us

	//// invert finger direction if enabled
	//if (_invert)
	//	_currPos = 1023 - _currPos;

	// calc positional error
	_currErr = (signed int)(_targPos - _currPos);

	// speed/position line gradient
	m = (float)(((float)_targSpeed) / ((float)proportionalOffset));

	// change the ± sign on the motorSpeed depending on required direction
	if (_currErr >= 0)
		vectorise = -1;

	// constrain speed to posError/speed graph
	if (abs(_currErr) < motorStopOffset)											// motor dead zone
	{
		motorSpeed = 0;
	}
	else if (_currErr > (signed int)(proportionalOffset + motorStopOffset))        // set to max speed depending on direction
	{
		motorSpeed = _targSpeed;
	}
	else if (_currErr < -(signed int)(proportionalOffset + motorStopOffset))		// set to -max speed depending on direction
	{
		motorSpeed = -_targSpeed;
	}
	else if (abs(_currErr) <= (proportionalOffset + motorStopOffset))				// proportional control
	{
		motorSpeed = (m * (_currErr + (motorStopOffset * vectorise))) - (_minSpeed * vectorise);
	}

	// constrain speed to limits
	motorSpeed = constrain(motorSpeed, -((signed int)_maxSpeed), (signed int)_maxSpeed);

	// if motor disabled, set speed to 0
	if (!_motorEn)
		motorSpeed = 0;

	// send speed to motors
	motorControl(motorSpeed);				// 15us
}
#endif

// split the vectorised motor speed into direction and speed values and write to the motor
void Finger::motorControl(signed int motorSpeed)
{
	static bool direction = 0;

	// split vectorised speed into speed and direction elements, and limit the results
	if (motorSpeed < (signed int)-_minSpeed)
	{
		(motorSpeed < (signed int)-_maxSpeed) ? motorSpeed = _maxSpeed : motorSpeed = -motorSpeed;
		direction = OPEN;
	}
	else if (motorSpeed >(signed int) _minSpeed)
	{
		(motorSpeed >(signed int) _maxSpeed) ? motorSpeed = _maxSpeed : motorSpeed;
		direction = CLOSE;
	}
	else motorSpeed = 0;

	// store current speed
	_currSpeed = motorSpeed;

#ifdef FORCE_SENSE
	static signed int prevMotorSpeed[MAX_FINGERS] = { 0 };
	static bool prevMotorDir[MAX_FINGERS] = { 0 };

	// if motor has just started a movement or direction has changed
	if (((prevMotorSpeed[fingerIndex] == 0) && (motorSpeed > 0)) ||
		(direction != prevMotorDir[fingerIndex]) ||
		((motorSpeed - prevMotorSpeed[fingerIndex]) > (MAX_FINGER_SPEED / 3)))
	{
		// start current spike timer
		currentSpikeTimer.start(CURR_SPIKE_DUR_US);
	}

	prevMotorSpeed[fingerIndex] = motorSpeed;
	prevMotorDir[fingerIndex] = direction;

#endif

	// 	invert finger direction if enabled
	if (_invert)
		direction = !direction;

	// write the speed to the motors
	analogWrite(_Pin.dir[direction], motorSpeed);   //write fingerSpeed to one direction pin
	analogWrite(_Pin.dir[!direction], 0);			//write 0 to other direction pin
}


#ifdef FORCE_SENSE

// stop the finger if the force limit is reached, or move to reach a target force
void Finger::forceController(void)
{
	// if force sense is not enabled, return
	if (!_forceSenseEn)
		return;

	US_NB_DELAY forceTimer;					// timer used to detect if the force is being sustained or is just a small peak
	const int force_pos_increment = 5;		// rate at which the actuator 'searches' for the target force

											// clear force limit flag
	_forceLimitFlag = false;

	// if force limit is reached
	if (_currForce > _maxForce)
	{
		MYSERIAL.print("F");
		MYSERIAL.print(fingerIndex);
		MYSERIAL.println(" max force");

		// store current movement direction so that stopping the motor is not perceived as a direction change
		bool tempDir = _currDir;

		// stop the motor by setting the target position to the current position
		writePos(readPos());

		// restore the previous direction, as it may have been changed by stopping the motor
		_currDir = tempDir;

		_forceLimitFlag = true;
	}

	// if a target force is set, move until force has been reached
	if (_targForce != 0)
	{
		// if target force limit has been reached
		if (_currForce > _targForce)
		{
			MYSERIAL.print("F");
			MYSERIAL.print(fingerIndex);
			MYSERIAL.println(" target force");

			_forceLimitFlag = true;
			_targForce = 0;
		}
		// if target force has not been reached
		else
		{
			// move pos in either the direction determined by _targForceDir in steps of size force_pos_increment
			movePos(-force_pos_increment + (_targForceDir * (2 * force_pos_increment)));
		}
	}
}

#endif

////////////////////////////// END OF FINGER CLASS //////////////////////////////


// runs the control() function of a Finger instance at each call by the timer interrupt
void _fingerControlInterrupt(void)
{
	static int i = 0;

	if (fingerISRList[i] != NULL)
	{
		fingerISRList[i]->control();		// run the control() member function of each attached Finger instance
	}

	i++;
	if (i > _TotalAttachedFingers)
		i = 0;
}


#ifdef FORCE_SENSE

// runs the readCurrentSns() function of a Finger instance at each call
void _currentSenseInterrupt(void)
{
	static int i = 0;

	if (fingerISRList[i] != NULL)
		fingerISRList[i]->readCurrentSns();		// run the readCurrentSns() member function of each attached Finger instance

	i++;
	if (i > _TotalAttachedFingers)
		i = 0;
}

#endif

#ifdef ARDUINO_AVR_MEGA2560
// change the PWM timer frequency to be out of the audible range
void setPWMFreq(uint8_t pin, uint8_t value)
{
	uint8_t timerNum;

	timerNum = PWM_pin_to_timer(pin);
	switch (timerNum)
	{
	case 0:
		//TCCR0B = (TCCR0B & 0xF8) | value;  // CHANGING THIS TIMER FREQUENCY WILL BREAK delay()
		break;
	case 1:
		TCCR1B = (TCCR1B & 0xF8) | value;
		break;
	case 2:
		TCCR2B = (TCCR2B & 0xF8) | value;
		break;
	case 3:
		TCCR3B = (TCCR3B & 0xF8) | value;
		break;
	case 4:
		TCCR4B = (TCCR4B & 0xF8) | value;
		break;
	case 5:
		TCCR5B = (TCCR5B & 0xF8) | value;
		break;
	default:
		break;
	}
}
#endif