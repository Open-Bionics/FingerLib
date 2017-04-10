/*
 * FingerLib.h
 *
 * Created: 02/11/2015 11:12:48
 * Author: Olly McBride
 *
 * This work is licensed under the Creative Commons Attribution 4.0 International License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/.
 *
 */ 

#ifndef FINGERLIB_H_
#define FINGERLIB_H_

#include <inttypes.h>
#include <Arduino.h>

#include "timers/timer_and_delay.h"

// CHANGE SETTINGS
// Uncomment out the following to use PID pos control instead of custom P control
#define USE_PID					

#if defined(ARDUINO_ARCH_SAMD)
// Uncomment the following to enable force/current sensing
#define FORCE_SENSE
#endif

// BOOLEANS
#define OPEN	0	
#define	CLOSE	1
#define RIGHT	1	// HAND TYPE
#define LEFT	2	// HAND TYPE

// LIMITS
#define MAX_FINGERS			6			// maximum number of _fingers
#define MAX_FINGER_SPEED	255			// maximum motor speed
#if defined(ARDUINO_AVR_MEGA2560)
#define MIN_FINGER_SPEED	180			// minimum motor speed
#else
#define MIN_FINGER_SPEED	210 // 0			// minimum motor speed
#endif
#define MAX_FINGER_POS		973			// maximum motor position
#define MIN_FINGER_POS		50			// minimum motor position
#define POS_REACHED_TOLERANCE	50		// tolerance for posReached()

// BOARD SPECIFIC LIBRARIES
#if defined(ARDUINO_AVR_MEGA2560)
	#ifndef MYSERIAL
		#define MYSERIAL Serial
	#endif
	#include "timers/avr_FingerTimer.h"
#elif defined(ARDUINO_ARCH_SAMD)
	#ifndef MYSERIAL
		#define MYSERIAL SerialUSB
	#endif
	#include "timers/samd_FingerTimer.h"

	#if defined(FORCE_SENSE)
		#include "current_sense/samd_CurrentSense.h"
	#endif
#else
	#error "FingerLib only supports boards using an Arduino Mega 2560 (Almond PCB) or an Arduino Zero (Chestnut/Chestnut+)."
#endif

// SETTING SPECIFIC LIBRARIES
#ifdef USE_PID
	#include "pid/pid_controller.h"
#endif 

#ifdef FORCE_SENSE
#define CURR_SPIKE_DUR_US	(double) 50000				// us - duration of peak to discard

// force and current conversion values
#define MOTOR_DRIVER_CURRENT_LIMIT		(float) 425		// mA
#define CURRENT_SENSE_SATURATION_VAL	(float) 950		// max ADC value
#define ADC_VALS_PER_MA_DRAW			(float) (CURRENT_SENSE_SATURATION_VAL/MOTOR_DRIVER_CURRENT_LIMIT)	// number of ADC values per mA draw of motor

// Y = MX + C, where Y = force, X = current draw
#define CURRENT_SENSE_CONST_M			(float) 7.4833	// generated from the equation of the line from the force-current graph
#define CURRENT_SENSE_CONST_C			(float) 46.444	// generated from the equation of the line from the force-current graph
#endif

// FINGER PINS
typedef struct _FingerPin
{
	uint8_t dir[2];
	uint8_t posSns;
#ifdef FORCE_SENSE
	uint8_t forceSns;
#endif
} FingerPin;


// FINGER CLASS
class Finger
{
	public:
		// CONSTRUCTOR
		Finger();

		// INITIALISATION
		uint8_t attach(uint8_t dir0, uint8_t dir1, uint8_t posSns);									// attach pins to a finger using only position control
		uint8_t attach(uint8_t dir0, uint8_t dir1, uint8_t posSns, bool inv);						// attach pins to a finger using only position control, but allow the direction to be inverted
		uint8_t attach(uint8_t dir0, uint8_t dir1, uint8_t posSns, uint8_t forceSns, bool inv);		// attach pins to a finger using position control and force control, and allow the direction to be inverted
		void detach(void);						// deactivate the finger
		bool attached(void);					// return true if the current finger is attached and initialised correctly 
		void invertFingerDir(void);				// set the motor to be inverted


		// LIMITS
		void setPosLimits(int min, int max);	// set the maximum and minimum position limits
		void setSpeedLimits(int min, int max);	// set the maximum and minimum speed limits
#ifdef FORCE_SENSE
		void setForceLimits(int min, int max);	// set the maximum and minimum force limits
#endif


		// POS
		void writePos(int value);			// write a target position to the finger
		void movePos(int value);			// write a change in position to the finger
		int16_t readPos(void);				// return the current position
		int16_t readPosError(void);			// return the error between the current position and the target position
		uint16_t readTargetPos(void);		// return the target position
		bool reachedPos(void);				// returns true if position reached
		bool reachedPos(uint16_t posErr);	// returns true if position reached


		// DIR
		void writeDir(int value);			// write a target direction to the finger
		uint8_t readDir(void);				// return the current direction
		void open(void);					// open the finger
		void close(void);					// close the finger
		void open_close(void);				// toggle finger between open/closed
		void open_close(boolean dir);		// set finger to open/close 


		// SPEED
		void writeSpeed(int value);			// write a target speed to the finger
		uint8_t readSpeed(void);			// return the current speed being written to the finger
		uint8_t readTargetSpeed(void);		// return the target speed


#ifdef FORCE_SENSE
		// FORCE
		void writeForce(float value, int dir);	// write a target force in a particular direction (0.0 - 54.95N, OPEN - CLOSE)
		float readForce(void);					// return the current force value. If force sense is not enabled, return blank (-1)
		uint16_t readCurrent(void);				// return the latest force sense ADC value
		bool reachedForceLimit(void);			// return true if the force limit has been reached
		void readCurrentSns(void);				// read the current force and discount the current spike (called via interrupt)
		float convertADCToForce(int ADCVal);	// convert ADC current sense value to force value (float)
		int convertForceToADC(float force);		// convert force value to ADC current sense value (int)
#endif

		// STOP/START
		void stopMotor(void);				// stop the motor and hold position
		void disableMotor(void);			// disable the motor by setting the speed to 0
		void enableMotor(void);				// re-enable the motor
		void motorEnable(bool motorEn);		// set motor to be enabled/disabled
#ifdef FORCE_SENSE
		void enableForceSense(void);		// enable force sensing
		void disableForceSense(void);		// disable force sensing
#endif
		

		// PRINT
		void printPos(void);				// print the current position (no new line)
		void printPos(bool newL);			// print the current position (new line)
		void printPosError(void);			// print the current position error (no new line)
		void printPosError(bool newL);		// print the current position error (new line)

		void printDir(void);				// print the current direction (no new line)
		void printDir(bool newL);			// print the current direction (new line)
		void printReached(void);			// print whether the target position has been reached (no new line)
		void printReached(bool newL);		// print whether the target position has been reached (new line)

		void printSpeed(void);				// print the current speed (no new line)
		void printSpeed(bool newL);			// print the current speed (new line)
				
		void printDetails(void);			// print current position, direction, speed and whether the target position has been reached
		void printConfig(void);				// print finger number, pins and limits


		// CONTROL
		void control(void);					// run position (and force) controller (called via interrupt)

	private:
	
#if defined(USE_PID)
		PID_CONTROLLER _PID;
#endif
		uint8_t fingerIndex;		// current finger number
		FingerPin _Pin;				// finger pin struct (dir, pos, force)
		bool _isActive = false;		// flag to indicate if finger is enabled
		bool _invert;				// finger inversion flag
		bool _motorEn;				// motor enable flag
#ifdef FORCE_SENSE
		bool _forceSenseEn = false;	// force sense is enable flag
#endif

		uint16_t _currPos;			// current position
		uint16_t _targPos;			// target position
		int16_t _currErr;			// position error
		uint16_t _minPos;			// position limit
		uint16_t _maxPos;			// position limit

		uint8_t _currDir;			// current direction

		int16_t _currSpeed;			// current speed
		uint16_t _targSpeed;		// target speed
		uint16_t _minSpeed;			// speed limit 
		uint16_t _maxSpeed;			// speed limit

#ifdef FORCE_SENSE
		uint16_t _currForce;		// current force (as ADC value)
		uint16_t _targForce;		// target force (as ADC value)
		bool _targForceDir;			// direction of force
		bool _forceLimitFlag;		// flag to indicate if force limit has been reached
		uint16_t _maxForce;			// force limit (as ADC value)
		uint16_t _minForce;			// force limit (as ADC value)
#endif

		void positionController(void);			// position controller (either PID or custom P)
		void motorControl(int motorSpeed);		// split the vectorised motor speed into direction and speed values and write to the motor
#ifdef FORCE_SENSE
		void forceController(void);				// stop the finger if the force limit is reached, or move to reach a target force

		US_NB_DELAY currentSpikeTimer;			// current spike rejection timer
#endif
};


// INTERRUPT HANDLERS
void _fingerControlInterrupt(void);		// runs the control() function of a Finger instance at each call by the timer interrupt
#ifdef FORCE_SENSE
void _currentSenseInterrupt(void);		// runs the readCurrentSns() function of a Finger instance at each call 
#endif

// HARDWARE SPECIFIC FUNCTIONS
#ifdef ARDUINO_AVR_MEGA2560
void setPWMFreq(uint8_t pin, uint8_t value);	// change the PWM timer frequency to be out of the audible range
#endif	

#endif /* FINGERLIB_H_ */