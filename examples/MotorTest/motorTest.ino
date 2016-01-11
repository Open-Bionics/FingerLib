/*  Open Bionics - FingerLib Example - MotorTest
* Author - Olly McBride
* Date - October 2015
*
* This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
* To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/.
* 
* Website - http://www.openbionics.com/
* GitHub - https://github.com/Open-Bionics
*/

#include <FingerLib.h>

/*
 * WHAT IT DOES
 * Reads the current position of each finger and prints the ADC values over Serial
 * Motors can be controlled/toggled (open/close) by entering the corresponding motor number (0 - 5)
 * Entering the number equivalent to the number of motors toggles all of the motors at once (6)
 * Entering any value greater than the number of motors stops all of the motors (>6)
 * 
 */
 

// MACROS
#define IS_BETWEEN(x,a,b) ((((x)>=(a))&&((x)<=(b)))?(1):(0))        // check if value x is between values a and b

// uncomment one of the following to select the board
//#define ADULT_BOARD
//#define FOUR_MOTOR_BOARD
//#define CHILD_BOARD
#define ALMOND_BOARD

// number of controllable fingers (number of motors)
#if defined(ADULT_BOARD)
#define NUM_FINGERS 5
#elif defined(FOUR_MOTOR_BOARD)
#define NUM_FINGERS 4
#elif defined(CHILD_BOARD)
#define NUM_FINGERS 3
#elif defined(ALMOND_BOARD)
#define NUM_FINGERS 6             // 6 motors instead of 5 as it is not hand (left/right) specific
#endif

// uncomment one of the following to select which hand is used
//int handFlag = LEFT;
int handFlag = RIGHT;

// initialise Finger class to array
Finger finger[NUM_FINGERS];

char* names[MAX_FINGERS] = {" m0"," m1"," m2"," m3"," m4"," m5"};   // motor names
int fingerNum = 0;

void setup()
{
  // MYSERIAL.print is used to allow compatability between both the Mega (Serial.print) 
  // and the Zero Native Port (SerialUSB.print), and is defined in FingerLib.h
  MYSERIAL.begin(38400);
  MYSERIAL.println("Serial Initialised");

  pinAssignment();
  MYSERIAL.println("Pins Initialised");
}

void loop()
{
  int i;
  if(MYSERIAL.available())
  {
    fingerNum = (int) MYSERIAL.read() - 48;
    if(IS_BETWEEN(fingerNum,0,(NUM_FINGERS-1)))
    {
      finger[fingerNum].open_close();     // toggle direction
    }
    else if(fingerNum == NUM_FINGERS) 
    {
      for(int i=0;i<NUM_FINGERS;i++)
      {
        finger[i].open_close();     // toggle direction 
      }
    }
    else if(fingerNum > NUM_FINGERS) 
    {
      stopMotors();
      MYSERIAL.println("MOTORS STOPPED");
      delay(500); 
    }
  }

  for(i=0;i<NUM_FINGERS;i++)
  {
    MYSERIAL.print(names[i]);
    MYSERIAL.print(" = ");
    MYSERIAL.print(finger[i].readPos());
    MYSERIAL.print("\t");
    delay(10);
  }
  MYSERIAL.println();

}

void stopMotors(void)
{
  int i;
  for(i=0;i<NUM_FINGERS;i++)
  {
    finger[i].stopMotor();
    //finger[i].writePos(finger[i].readPos());
  }
}
