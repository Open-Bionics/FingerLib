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
	if (_fingerIndex >= MAX_FINGERS)			// if too many fingers have been initialised
	{
		_fingerIndex = -1;						// set current finger number to be empty
		_isActive = false;						// set current finger as inactive 
	}
	else
	{
		_fingerIndex = _TotalFingerCount++;		// count the total number of fingers initialised 
	}

	motorEnable(false);

	_pos = { 0 };
	_dir = { 0 };
	_speed = { 0 };
	_PWM = { 0 };
#ifdef FORCE_SENSE					
	_force = { 0 };
#endif

	_interruptEn = true;				// use the timer interrupt by default
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
	if ((_fingerIndex >= 0) && (_fingerIndex < MAX_FINGERS))
	{
		motorEnable(false);				// disable the motor
		
		// attach all finger pins
		_pin.dir[0] = dir0;
		_pin.dir[1] = dir1;
		_pin.posSns = posSns;
#ifdef FORCE_SENSE
		_pin.forceSns = forceSns;
#endif
		_invert = inv;					// store whether to invert the finger direction
		
		// configure pins
		pinMode(dir0, OUTPUT);		// set direction1 pin to output
		pinMode(dir1, OUTPUT);		// set direction2 pin to output
		pinMode(posSns, INPUT);		// set position sense pin to input
#ifdef FORCE_SENSE
		pinMode(forceSns, INPUT);	// set force sense pin to input
#endif

		// initialise circle buffer
		_velBuff.begin(VEL_BUFF_SIZE);
#if	defined(FORCE_SENSE)
		_IBuff.begin(CURR_SENSE_BUFF_SIZE);
#endif

		// set limits and initial values
		setPosLimits(MIN_FINGER_POS, MAX_FINGER_POS);
		setPWMLimits(MIN_FINGER_PWM, MAX_FINGER_PWM);
		writeDir(OPEN);
		writeSpeed(MAX_FINGER_PWM);
#ifdef FORCE_SENSE
		forceSenseEnable(true);
#endif

		// if finger is being attached for the first time
		if (!_isActive)
		{
			_TotalAttachedFingers++;					// update the count of total attached fingers
		}

		// add a pointer to the current finger instance to the list for position control calls, used by the ISR
		fingerISRList[_fingerIndex] = this;

		// initialise the position control timer
		if (!_posCtrlTimerInit)
		{
			if (_interruptEn)							// if the interrupt is enabled for motor control
			{
				_passMotorPtr(&_fingerControlCallback);	// attach the finger control function to the timer
			}

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

		motorEnable(true);				// re-enable the motor
		_isActive = true;				// set the current finger to be active
		return _fingerIndex;			// return the current finger number
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
void Finger::setPWMLimits(int min, int max)
{
	// set limits
	_PWM.limit.min = min;
	_PWM.limit.max = max;

#if	defined(USE_PID)
	_PID.setLimits(-(double)_PWM.limit.max, _PWM.limit.max);
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
	_pos.error = (_pos.targ - _pos.curr);

	//determine direction of travel
	if ((uint16_t)value > _pos.curr)
	{
		_dir.targ = CLOSE;
	}
	else
	{
		_dir.targ = OPEN;
	}
}

// write a change in position to the finger
void Finger::movePos(int value)
{
	// change position
	_pos.targ += readPos() + value;

	// constrain position value to limits
	_pos.targ = constrain((uint16_t)_pos.targ, _pos.limit.min, _pos.limit.max);

	// calculate new position error (to remove false positives in reachedPos() )
	_pos.error = (_pos.targ - _pos.curr);
}

// return the current position
int16_t Finger::readPos(void)
{
	// read finger position
	noInterrupts();
	_pos.curr = analogRead(_pin.posSns);

	// invert finger direction if enabled
	if (_invert)
	{
		_pos.curr = MAX_POS_SENSOR_VAL - _pos.curr;
	}
	interrupts();

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
	{
		return true;
	}
	else
	{
		return false;
	}
}

// returns true if position reached
bool Finger::reachedPos(uint16_t posErr)
{
	// if the position error is within a tolerance
	if (abs(readPosError()) < (int16_t)posErr)
	{
		return true;
	}
	else
	{
		return false;
	}
}


// DIR

// write a target direction to the finger
void Finger::writeDir(int value)
{
	// store direction
	_dir.targ = value;

	// set new target position based on input direction
	if (_dir.targ == OPEN)
	{
		_pos.targ = _pos.limit.min;
	}
	else if (_dir.targ == CLOSE)
	{
		_pos.targ = _pos.limit.max;
	}
}

// return the current direction
uint8_t Finger::readDir(void)
{
	return _dir.targ;
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
	{
		open();
	}
	else if (dir == CLOSE)
	{
		close();
	}
}


// SPEED

// write a target speed to the finger
void Finger::writeSpeed(int value)
{
	_speed.targ = constrain((int16_t)value, -_PWM.limit.max, _PWM.limit.max);		// store vectorised speed
	_PWM.targ = _speed.targ;

#if	defined(USE_PID)
	_PID.setLimits(-abs(_PWM.targ), abs(_PWM.targ));
#endif
}

// return the current movement speed
float Finger::readSpeed(void)
{
	//pauseInterrupt();				// pause 'control()' interrupt to prevent a race condition

	calcVel();						// read finger speed (ADC/per)
	_velBuff.write(_speed.raw);
	_speed.prev = _speed.curr;
	_speed.curr = _velBuff.readMean();

	//resumeInterrupt();			// resume 'control()' interrupt to prevent a race condition

	return _speed.raw;
}

// return the target movement speed
float Finger::readTargetSpeed(void)
{
	return _speed.targ;
}

// return the current speed being written to the finger
int Finger::readPWM(void)
{
	return _PWM.curr;
}

// return the target speed
int Finger::readTargetPWM(void)
{
	return _PWM.targ;
}

#ifdef FORCE_SENSE
// FORCE

// return the current force value. If force sense is disabled, return blank (-1)
float Finger::readForce(void)
{
	// if force sense is not enabled, return
	if (!_forceSnsEn)
	{
		return (-1);
	}
	else
	{
		return convertADCToForce(readCurrent());
	}
}

// return the latest force sense ADC value
uint16_t Finger::readCurrent(void)
{
	// if force sense is not enabled, return
	if (!_forceSnsEn)
	{
		return 0;
	}
	else
	{
		return _IBuff.readMean();
	}
}

// return true if the force limit has been reached
bool Finger::reachedForceLimit(void)
{
	return _force.limit.reached;
}

// read the current force and discount the current spike
void Finger::calcCurrentSns(void)
{
	// if still within current spike period
	if (_currentSpikeTimer.started() && !_currentSpikeTimer.finished())
	{
		_force.curr = 0;
	}
	else
	{
		noInterrupts();
		_force.curr = analogRead(_pin.forceSns);
		interrupts();
	}

	// store the read current into a buffer for smoothing
	_IBuff.write((uint16_t)_force.curr);
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


// ENABLE/DISABLE

// set motor to be enabled/disabled
void Finger::motorEnable(bool en)
{
	_motorEn = en;

	if (_motorEn)
	{
		_PID.reset();
	}
}

// return true if the motor is enabled
bool Finger::enabled(void)
{
	return _motorEn;
}

#ifdef FORCE_SENSE
// set force sensing to be enabled/disabled
void Finger::forceSenseEnable(bool en)
{
	_forceSnsEn = en;
}
#endif

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
//	MYSERIAL.print(_fingerIndex);
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
//	MYSERIAL.print(_fingerIndex);
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
//	MYSERIAL.print(_PWM.limit.min);
//	MYSERIAL.print("\tMaxSpeed: ");
//	MYSERIAL.println(_PWM.limit.max);
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
	// read finger position (reads to _pos.curr internally() )
	readPos();

	// read finger speed (including calculating the vel)
	readSpeed();

//#ifdef FORCE_SENSE
//	// if force sense is enabled, run force control
//	if (_forceSnsEn)
//	{
//		forceController();
//	}
//
//#endif


#ifdef FORCE_SENSE
	if (_forceSnsEn && _motorEn)
	{
		stallDetection();
	}
#endif
	

	positionController();	// run the position controller 
}










////////////////////////////// Private Methods //////////////////////////////

#if defined(USE_PID)
// position controller (using PID)
void Finger::positionController(void)
{
	// run pos PID controller to calculate target speed
	_speed.targ = _PID.run(_pos.targ, _pos.curr);

	motorControl(_speed.targ);
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
	m = (float)(((float)_PWM.targ) / ((float)proportionalOffset));

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
		motorSpeed = _PWM.targ;
	}
	else if (_pos.error < -(signed int)(proportionalOffset + motorStopOffset))		// set to -max speed depending on direction
	{
		motorSpeed = -_PWM.targ;
	}
	else if (abs(_pos.error) <= (proportionalOffset + motorStopOffset))				// proportional control
	{
		motorSpeed = (m * (_pos.error + (motorStopOffset * vectorise))) - (_PWM.limit.min * vectorise);
	}

	// constrain speed to limits
	motorSpeed = constrain(motorSpeed, -((signed int)_PWM.limit.max), (signed int)_PWM.limit.max);

	// if motor disabled, set speed to 0
	if (!_motorEn)
		motorSpeed = 0;

	// send speed to motors
	motorControl(motorSpeed);				// 15us
}
#endif

// split the vectorised motor speed into direction and speed values and write to the motor
void Finger::motorControl(signed int speed)
{
	bool dir = OPEN;

	// if the motor is disabled, set the speed to 0
	if (!_motorEn)
	{
		speed = 0;
	}
	
	// determine direction and invert speed if necessary
	if (speed > 0)
	{
		dir = OPEN;
	}
	else if (speed < 0)
	{
		dir = CLOSE;
		speed = -speed;
	}

	// if speed is not within min/max, set to 0
	speed = window(speed, 0, _PWM.limit.max);

	// store previous and current speed
	_PWM.prev = _PWM.curr;
	_PWM.curr = (uint8_t)speed;

	// store previous and current direction
	_dir.prev = _dir.curr;
	_dir.curr = dir;

#ifdef FORCE_SENSE
	// if motor has just started a movement or direction has changed or the PWM speed has changed by > 1/3 of the max
	if (((_PWM.prev == 0) && (_PWM.curr > 0)) || (_dir.curr != _dir.prev) || ((_PWM.curr - _PWM.prev) > (MAX_FINGER_PWM / 3)))
	{
		// start current spike timer
		_currentSpikeTimer.start(CURR_SPIKE_DUR_US);
	}
#endif

	// 	invert finger direction if enabled
	if (_invert)
	{
		dir = !dir;
	}



	// write the speed to the motors
	analogWrite(_pin.dir[dir], speed);		//write motor speed to one direction pin
	analogWrite(_pin.dir[!dir], 0);			//write 0 to other direction pin
	
}


#ifdef FORCE_SENSE

bool Finger::stallDetection(void)
{
	// if the motor is stopped and drawing a lot of current, set the targ pos to be the curr pos
	if ((abs(_speed.curr) <= 1) && (_IBuff.readMean() >= STALL_CURRENT_THRESH))
	{
		if (!_motorStallTimer.started())
		{
			_motorStallTimer.start(MAX_STALL_TIME_MS);
		}
		else if (_motorStallTimer.finished())	// if the motor has been stalled for the MAX_STALL_TIME_MS
		{
			_motorStallTimer.stop();

			// hold the motor at the current pos
			_pos.targ = _pos.curr;
			_pos.error = (_pos.targ - _pos.curr);

//#define MYSERIAL SerialUSB
//			MYSERIAL.print("\nF");
//			MYSERIAL.print(_fingerIndex);
//			MYSERIAL.print(": stall detected. Setting to ");
//			MYSERIAL.println(_pos.targ);
//			MYSERIAL.print("\n");

			return true;
		}
	}
	else
	{
		_motorStallTimer.stop();
	}

	return false;
}

// stop the finger if the force limit is reached, or move to reach a target force
void Finger::forceController(void)
{
	// if force sense is not enabled, return
	if (!_forceSnsEn)
	{
		return;
	}

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
	//		MYSERIAL.print(_fingerIndex);
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



// calculate the velocity of the motor
void Finger::calcVel(void)
{	
	// if the duration timer is currently running
	//if (_velTimer.started())
	//if (_velTimer.now() > 100000)		// 100ms
	if (_velTimer.now() > 50000)		// 50ms
	{
		// get the time and position at the current time
		double duration = _velTimer.now();		// us. time since last vel calc
		double pos = _pos.curr;					// store current pos to prevent race condition
		double dist = (pos - _pos.prev);		// calculate distance moved (using stored pos)

		if (abs(dist) > 0)			// if there has been movement
		{
			_speed.raw = (dist * (double)100000) / duration;		// calc vel in ADC ticks per s?
		}
		else						// else if 0 distance has moved
		{
			_speed.raw = 0.0;		// set the velocity to 0
		}

		_velTimer.start();			// restart timer
		_pos.prev = pos;			// save previous pos
	}
	else if (!_velTimer.started())	// if the vel timer is not currently running
	{
		_velTimer.start();			// start the vel timer
		_speed.raw = 0;				// clear the velocity	
	}
}

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
	{
		i = 0;
	}
}


#ifdef FORCE_SENSE

// runs the calcCurrentSns() function of a Finger instance at each call
void _currentSenseCallback(void)
{
	static int i = 0;

	if (fingerISRList[i] != NULL)
	{
		fingerISRList[i]->calcCurrentSns();		// run the calcCurrentSns() member function of each attached Finger instance
	}

	i++;
	if (i > _TotalAttachedFingers)
	{
		i = 0;
	}
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