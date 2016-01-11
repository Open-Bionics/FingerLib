/*  Open Bionics - FingerLib Example - MultipleFingers
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
 * Uses the FingerLib.h library to open and close all fingers and thumb every 2 seconds
 * Attaches the finger pins in the file 'Pins.ino'
 * 
 */
 
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
#define NUM_FINGERS 5
#endif

// uncomment one of the following to select which hand is used
//int handFlag = LEFT;
int handFlag = RIGHT;

// initialise Finger class to array
Finger finger[NUM_FINGERS];

void setup()
{
  // MYSERIAL.print is used to allow compatability between both the Mega (Serial.print) 
  // and the Zero Native Port (SerialUSB.print), and is defined in FingerLib.h
  MYSERIAL.begin(38400);
  MYSERIAL.println("Started");

  // configure all of the finger pins 
  pinAssignment();
  MYSERIAL.println("Pins configured");

}

void loop()
{
  // set all of the fingers to OPEN
  openHand();
  MYSERIAL.println("Opening");
  // give them time to OPEN
  delay(2000);
  // set all of the fingers to CLOSE
  closeHand();
  MYSERIAL.println("Closing");
  // give them time to CLOSE
  delay(2000);
}

// count through all the fingers and set them to open
void openHand(void)
{
  int i;
  for(i=0;i<NUM_FINGERS;i++)
  {
    finger[i].open();
  }
}

// count through all the fingers and set them to close
void closeHand(void)
{
  int i;
  for(i=0;i<NUM_FINGERS;i++)
  {
    finger[i].close();
  }
}

