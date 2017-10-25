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
		fingerIndex = -1;						// set current finger number to be empty
		_isActive = false;						// set current finger as inactive 
	}
	else
	{
		fingerIndex = _TotalFingerCount++;		// count the total number of fingers initialised 
	}

	_interruptEn = true;		// use the timer interrupt by default


#ifdef FORCE_SENSE			// if force sense is available on the current board, initialise the values		
	_force.limit.reached = 0;
	_force.targ = 0;
	_force.curr = 0;			// force values converted to ADC values for quicker maths
	_force.limit.max = 1023;		// force values converted to ADC values for quicker maths
	_force.limit.min = 0;			// force values converted to ADC values for quicker maths
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
		_pin.dir[0] = dir0;
		_pin.dir[1] = dir1;
		_pin.posSns = posSns;
#ifdef FORCE_SENSE
		_pin.forceSns = forceSns;
#endif

		// set limits and initial values
		setPosLimits(MIN_FINGER_POS, MAX_FINGER_POS);
		setSpeedLimits(MIN_FINGER_SPEED, MAX_FINGER_SPEED);
		writeSpeed(MAX_FINGER_SPEED);
		writePos(MIN_FINGER_POS);

		// enable the motor and disable finger inversion 
		_invert = inv;					// store whether to invert the finger direction
		_motorEn = true;				// enable the motor
		_dir.curr = OPEN;				// set dir to OPEN after initial writePos to configure finger dir

		// if finger is being attached for the first time
		if (!_isActive)
			_TotalAttachedFingers++;	// update the count of total attached fingers

		// add a pointer to the current finger instance to the list for position control calls, used by the ISR
		fingerISRList[fingerIndex] = this;

		// initialise the position control timer
		if (!_posCtrlTimerInit)
		{
			if(_interruptEn)							// if the interrupt is enabled for motor control
				_passMotorPtr(&_fingerControlCallback);	// attach the finger control function to the timer

			_posCtrlTimerSetup();						// initialise and start the timer
			_posCtrlTimerInit = true;					
		}

#ifdef FORCE_SENSE
		// attach current sense control interrupt, initialise and sync PWM timers
		initCurrentSense(_pin.dir[0], _pin.dir[1], &_currentSenseCallback);
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
		//MYSERIAL.println("ERROR - Too many fingers attached");
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
	_pos.limit.min = min;
	_pos.limit.max = max;
}

// set the maximum and minimum speed limits
void Finger::setSpeedLimits(int min, int max)
{
	// set limits
	_speed.limit.min = min;
	_speed.limit.max = max;

#if	defined(USE_PID)
	_PID.setLimits(-(double)_speed.limit.max, _speed.limit.max);
#endif
}

#ifdef FORCE_SENSE
// set the maximum and minimum force limits
void Finger::setForceLimits(int min, int max)
{
	// set limits
	_force.limit.min = convertForceToADC(min);				// force values converted to ADC values for quicker maths
	_force.limit.max = convertForceToADC(max);				// force values converted to ADC values for quicker maths
}
#endif


// POS

// write a target position to the finger
void Finger::writePos(int value)
{
	// constrain position value to limits
	_pos.targ = constrain((uint16_t)value, _pos.limit.min, _pos.limit.max);

	// calculate new position error (to remove false positives in reachedPos() )
	_pos.error = (signed int)(_pos.targ - _pos.curr);

	//determine direction of travel
	if ((uint16_t)value > _pos.curr)
		_dir.curr = CLOSE;
	else
		_dir.curr = OPEN;
}

// write a change in position to the finger
void Finger::movePos(int value)
{
	// change position
	_pos.targ += value;

	// constrain position value to limits
	_pos.targ = constrain((uint16_t)_pos.targ, _pos.limit.min, _pos.limit.max);

	// calculate new position error (to remove false positives in reachedPos() )
	_pos.error = (signed int)(_pos.targ - _pos.curr);
}

// return the current position
int16_t Finger::readPos(void)
{
	return _pos.curr;
}

// return the error between the current position and the target position
int16_t Finger::readPosError(void)
{
	return _pos.error;
}

// return the target position
uint16_t Finger::readTargetPos(void)
{
	return _pos.targ;
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
	_dir.curr = value;

	// set new target position based on input direction
	if (_dir.curr == OPEN)
		_pos.targ = _pos.limit.min;
	else if (_dir.curr == CLOSE)
		_pos.targ = _pos.limit.max;
}

// return the current direction
uint8_t Finger::readDir(void)
{
	return _dir.curr;
}

// open the finger
void Finger::open(void)
{
	writePos(_pos.limit.min);
}

// close the finger
void Finger::close(void)
{
	writePos(_pos.limit.max);
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
	_speed.targ = constrain((uint16_t)value, _speed.limit.min, _speed.limit.max);

#if	defined(USE_PID)
		_PID.setLimits(-(double)_speed.targ, _speed.targ);
#endif
}

// return the current speed being written to the finger
uint8_t Finger::readSpeed(void)
{
	return _speed.curr;
}

// return the target speed
uint8_t Finger::readTargetSpeed(void)
{
	return _speed.targ;
}


#ifdef FORCE_SENSE
// FORCE

//// write a target force in a particular direction (0.0 - 54.95N, OPEN - CLOSE)
//void Finger::writeForce(float value, int dir)
//{
//	_force.targ = convertForceToADC(value);		// convert force value (float) to ADC value (uint16_t) for quicker maths
//	_targForceDir = dir;						// store the target force direction
//	_force.limit.reached = false;					// clear limit flag when a new limit is set
//}

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
		return _force.curr;
}

// return true if the force limit has been reached
bool Finger::reachedForceLimit(void)
{
	return _force.limit.reached;
}

// read the current force and discount the current spike
void Finger::readCurrentSns(void)
{
	// if still within current spike period
	if (currentSpikeTimer.started() && !currentSpikeTimer.finished())
		_force.curr = 0;
	else
		_force.curr = analogRead(_pin.forceSns);
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

// enable timer interrupt for motor control
void Finger::enableInterrupt(void)
{
	_passMotorPtr(&_fingerControlCallback);	// attach the finger control function to the timer

	_interruptEn = true;
}

// disable timer interrupt for motor control
void Finger::disableInterrupt(void)
{
	_passMotorPtr(NULL);						// prevent the interrupt from calling the motor control function
	
	_interruptEn = false;
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

//// print the current position (no new line)
//void Finger::printPos(void)
//{
//	printPos(0);
//}
//
//// print the current position (new line)
//void Finger::printPos(bool newL)
//{
//	MYSERIAL.print("Pos ");
//	MYSERIAL.print(readPos());
//	MYSERIAL.print("  ");
//	if (newL)
//		MYSERIAL.print("\n");
//}
//
//// print the current position error (no new line)
//void Finger::printPosError(void)
//{
//	printPosError(0);
//}
//
//// print the current position error (new line)
//void Finger::printPosError(bool newL)
//{
//	MYSERIAL.print("Err ");
//	MYSERIAL.print(readPosError());
//	MYSERIAL.print("  ");
//	if (newL)
//		MYSERIAL.print("\n");
//}
//
//// print the current direction (no new line)
//void Finger::printDir(void)
//{
//	printDir(0);
//}
//
//// print the current direction (new line)
//void Finger::printDir(bool newL)
//{
//	const char* _dirString[2] = { "OPEN","CLOSE" };	// direction string
//
//	MYSERIAL.print("Dir ");
//	MYSERIAL.print(_dirString[readDir()]);
//	MYSERIAL.print("  ");
//	if (newL)
//		MYSERIAL.print("\n");
//}
//
//// print whether the target position has been reached (no new line)
//void Finger::printReached(void)
//{
//	printReached(0);
//}
//
//// print whether the target position has been reached (new line)
//void Finger::printReached(bool newL)
//{
//	MYSERIAL.print("Reached ");
//	MYSERIAL.print(reachedPos());
//	MYSERIAL.print("  ");
//	if (newL)
//		MYSERIAL.print("\n");
//}
//
//// print the current speed (no new line)
//void Finger::printSpeed(void)
//{
//	printSpeed(0);
//}
//
//// print the current speed (new line)
//void Finger::printSpeed(bool newL)
//{
//	MYSERIAL.print("Speed ");
//	MYSERIAL.print(readSpeed());
//	MYSERIAL.print("  ");
//	if (newL)
//		MYSERIAL.print("\n");
//}
//
//// print current position, direction, speed and whether the target position has been reached
//void Finger::printDetails(void)
//{
//	MYSERIAL.print("Finger ");
//	MYSERIAL.print(fingerIndex);
//	MYSERIAL.print("  ");
//	printPos();
//	printDir();
//	printSpeed();
//	printReached(true);		// print new line after
//
//}
//
//// print finger number, pins and limits
//void Finger::printConfig(void)
//{
//	MYSERIAL.print("Finger");
//	MYSERIAL.print(fingerIndex);
//	MYSERIAL.print(" -");
//	MYSERIAL.print(" \tdir0: ");
//	MYSERIAL.print(_pin.dir[0]);
//	MYSERIAL.print(" \tdir1: ");
//	MYSERIAL.print(_pin.dir[1]);
//	MYSERIAL.print(" \tposSense: ");
//	MYSERIAL.print(_pin.posSns);
//#ifdef FORCE_SENSE
//	MYSERIAL.print(" \tforceSense: ");
//	MYSERIAL.print(_pin.forceSns);
//#endif
//	MYSERIAL.print("\tinvert: ");
//	MYSERIAL.println(_invert);
//
//	MYSERIAL.print("MinPos: ");
//	MYSERIAL.print(_pos.limit.min);
//	MYSERIAL.print("\tMaxPos: ");
//	MYSERIAL.println(_pos.limit.max);
//
//	MYSERIAL.print("MinSpeed: ");
//	MYSERIAL.print(_speed.limit.min);
//	MYSERIAL.print("\tMaxSpeed: ");
//	MYSERIAL.println(_speed.limit.max);
//
//#ifdef FORCE_SENSE
//	MYSERIAL.print("MinForce: ");
//	MYSERIAL.print(_force.limit.min);
//	MYSERIAL.print("\tMaxForce: ");
//	MYSERIAL.println(_force.limit.max);
//#endif
//
//
//	MYSERIAL.print("\n");
//}


//
void Finger::control(void)
{
	// read finger position
	noInterrupts();
	_pos.curr = analogRead(_pin.posSns);
	interrupts();

	// invert finger direction if enabled
	if (_invert)
		_pos.curr = 1023 - _pos.curr;

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

	// if motor enabled
	if (_motorEn)
		speed = _PID.run(_pos.targ, _pos.curr);

	// DEBUG
	//MYSERIAL.print("F");
	//MYSERIAL.print(fingerIndex);
	//MYSERIAL.print(": C: ");
	//MYSERIAL.print(_pos.curr);
	//MYSERIAL.print("  T: ");
	//MYSERIAL.print(_pos.targ);
	//MYSERIAL.print("  S: ");
	//MYSERIAL.println(speed);
	//if (fingerIndex >= 3)
	//	MYSERIAL.println("\n");


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
	int proportionalOffset = 300;
	signed int motorStopOffset = 25;
#elif defined(ARDUINO_ARCH_SAMD)
	int proportionalOffset = 300;
	signed int motorStopOffset = 20;
#endif

	// calc positional error
	_pos.error = (signed int)(_pos.targ - _pos.curr);

	// speed/position line gradient
	m = (float)(((float)_speed.targ) / ((float)proportionalOffset));

	// change the ± sign on the motorSpeed depending on required direction
	if (_pos.error >= 0)
		vectorise = -1;

	// constrain speed to posError/speed graph
	if (abs(_pos.error) < motorStopOffset)											// motor dead zone
	{
		motorSpeed = 0;
	}
	else if (_pos.error > (signed int)(proportionalOffset + motorStopOffset))        // set to max speed depending on direction
	{
		motorSpeed = _speed.targ;
	}
	else if (_pos.error < -(signed int)(proportionalOffset + motorStopOffset))		// set to -max speed depending on direction
	{
		motorSpeed = -_speed.targ;
	}
	else if (abs(_pos.error) <= (proportionalOffset + motorStopOffset))				// proportional control
	{
		motorSpeed = (m * (_pos.error + (motorStopOffset * vectorise))) - (_speed.limit.min * vectorise);
	}

	// constrain speed to limits
	motorSpeed = constrain(motorSpeed, -((signed int)_speed.limit.max), (signed int)_speed.limit.max);

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
	if (motorSpeed < (signed int)-_speed.limit.min)
	{
		(motorSpeed < (signed int)-_speed.limit.max) ? motorSpeed = _speed.limit.max : motorSpeed = -motorSpeed;
		direction = OPEN;
	}
	else if (motorSpeed >(signed int) _speed.limit.min)
	{
		(motorSpeed >(signed int) _speed.limit.max) ? motorSpeed = _speed.limit.max : motorSpeed;
		direction = CLOSE;
	}
	else motorSpeed = 0;

	// store current speed
	_speed.curr = motorSpeed;

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
	analogWrite(_pin.dir[direction], motorSpeed);   //write fingerSpeed to one direction pin
	analogWrite(_pin.dir[!direction], 0);			//write 0 to other direction pin
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
	_force.limit.reached = false;

	// if force limit is reached
	if (_force.curr > _force.limit.max)
	{
		// store current movement direction so that stopping the motor is not perceived as a direction change
		bool tempDir = _dir.curr;

		// stop the motor by setting the target position to the current position
		writePos(readPos());

		// restore the previous direction, as it may have been changed by stopping the motor
		_dir.curr = tempDir;

		_force.limit.reached = true;
	}

	//// if a target force is set, move until force has been reached
	//if (_force.targ != 0)
	//{
	//	// if target force limit has been reached
	//	if (_force.curr > _force.targ)
	//	{
	//		MYSERIAL.print("F");
	//		MYSERIAL.print(fingerIndex);
	//		MYSERIAL.println(" target force");

	//		_force.limit.reached = true;
	//		_force.targ = 0;
	//	}
	//	// if target force has not been reached
	//	else
	//	{
	//		// move pos in either the direction determined by _targForceDir in steps of size force_pos_increment
	//		movePos(-force_pos_increment + (_targForceDir * (2 * force_pos_increment)));
	//	}
	//}
}

#endif

////////////////////////////// END OF FINGER CLASS //////////////////////////////


// runs the control() function of a Finger instance at each call by the timer interrupt
void _fingerControlCallback(void)
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
void _currentSenseCallback(void)
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