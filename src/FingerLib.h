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

// Architecture specific include
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
#else
#error "This library only supports boards an Arduino Mega 2560 or an Arduino Zero."
#endif

#define OPEN	0
#define	CLOSE	1
#define RIGHT	1	// handFlag
#define LEFT	2	// handFlag

#define MAX_FINGERS			6			// maximum number of _fingers
#define MAX_FINGER_SPEED	255			// maximum motor speed
#define MIN_FINGER_SPEED	99			// minimum motor speed
#define MAX_FINGER_POS		923			// maximum motor position
#define MIN_FINGER_POS		150			// minimum motor position
#define POS_REACHED_TOLERANCE	50		// tolerance for posReached()

typedef struct  {
	uint8_t dir[2];
	uint8_t sns;
	uint8_t isActive	:1 ;             // true if this channel is enabled
} FingerPin_t;

typedef struct {
	FingerPin_t Pin;
	uint8_t CurrDir;
	uint16_t TargSpeed;
	int16_t CurrSpeed;
	uint16_t MinSpeed;
	uint16_t MaxSpeed;
	uint16_t TargPos;
	uint16_t CurrPos;
	int16_t CurrErr;
	uint16_t MinPos;
	uint16_t MaxPos;
	bool motorEn;
	bool invert;
} finger_t;

class Finger
{
	public:
		Finger();
		//Finger(int board, int left_right);
		uint8_t attach(int dir0, int dir1, int sense);				// attach direction & sense pins of a finger 
		uint8_t attach(int dir0, int dir1, int sense, bool inv);
		void detach(void);
		bool attached(void);                   
		void writePos(int value);   
		void writeDir(int value);   
		void writeSpeed(int value);   
		int16_t readPos(void);
		int16_t readPosError(void);
		uint16_t readTargetPos(void);
		uint8_t readDir(void);
		uint8_t readSpeed(void);
		uint8_t readTargSpeed(void);
		void setPosLimits(int min, int max);
		void setSpeedLimits(int min, int max); 
		void invertFingerDir(void);
		
		void stopMotor(void);				// stop single motor
		void disableMotor(void);			// set motor speed to 0
		void enableMotor(void);				// set motor speed to 0
		bool reachedPos(void);				// returns if position reached
		bool reachedPos(uint16_t posErr);	// returns if position reached
		void open(void);					// open finger to min position
		void close(void);					// close finger to max position
		void open_close(void);				// toggle finger between open/closed
		void open_close(boolean dir);		// set finger to fully open/closed 
		
		void printSpeed(void);
		void printSpeed(int newL);
		void printPos(void);
		void printPos(int newL);
		void printPosError(void);
		void printPosError(int newL);
		void printDir(void);
		void printDir(int newL);
		void printReached(void);
		void printReached(int newL);
		void printDetails(void);
		
	private:    
 		uint8_t fingerIndex;			// index into the channel data for this finger
		
};

void motorControl(int fNum, signed int motorSpeed);
void fingerPosCtrl(void);

#endif /* FINGERLIB_H_ */