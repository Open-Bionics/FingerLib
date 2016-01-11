/*  Open Bionics - FingerLib Example - HandDemo
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
 * Runs through a demo sequence using FingerLib.h to control the fingers
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

  // set hand to the open position and wait for it to open
  openHand();
  delay(1500);

}

void loop()
{
  MYSERIAL.println("Finger Roll");
  fingerRoll();
  fingerRoll();
  delay(1000);

  MYSERIAL.println("Thumbs Up");
  thumbsUp();
  thumbsUp();
  delay(1000);

  MYSERIAL.println("Open");
  openHand();
  delay(1000);

  MYSERIAL.println("Pinch");
  pinch();
  pinch();
  delay(1000);
}

// count through all the fingers and set them to open
void openHand(void)
{
  int i;
  for (i = 0; i < NUM_FINGERS; i++)
  {
    finger[i].open();
  }
}

// count through all the fingers and set them to close
void closeHand(void)
{
  int i;
  for (i = 0; i < NUM_FINGERS; i++)
  {
    finger[i].close();
  }
}

bool allFingersClosed(void)
{
  int errPos = 380;
  if (finger[0].reachedPos(errPos) &&
      finger[1].reachedPos(errPos) &&
      finger[2].reachedPos(errPos) &&
      finger[3].reachedPos(errPos) &&
      finger[4].reachedPos(errPos))
    return 1;
  else
    return 0;
}

// rolls the fingers closed and open, the next finger starts moving when the previous finger reaches it's target position
void fingerRoll(void)
{
  int i;
  MYSERIAL.println("Fingers rolling closed");
  //for (i = 0; i < NUM_FINGERS; i++)
  for (i = NUM_FINGERS-1; i >= 0; i--)
  {
    finger[i].close();                   // set finger to close
    while (!finger[i].reachedPos(390));  // wait for finger to be fully closed
    // reachedPos(val) returns true when the error between the current and target position is less than 'val' 
  }
  MYSERIAL.println("Fingers rolling open");
  //for (i = 0; i < NUM_FINGERS; i++)
  for (i = NUM_FINGERS-1; i >= 0; i--)
  {
    finger[i].open();                   // set finger to open
    while (!finger[i].reachedPos(20));  // wait for finger to be fully open
    // reachedPos(val) returns true when the error between the current and target position is less than 'val' 
  }
}

// the next finger starts moving when the previous finger reaches it's target position
void thumbsUp(void)
{
  MYSERIAL.println("Close all fingers");
  closeHand();
  while (!allFingersClosed());      // wait for all fingers to be fully closed

  MYSERIAL.println("Wait for thumb to open");
  finger[0].open();                   // set thumb to open
  while (!finger[0].reachedPos(20));  // wait for thumb to be fully open

  MYSERIAL.println("Wait 1s with thumb open");
  delay(1000);        // wait for a bit

  MYSERIAL.println("Close thumb");
  finger[0].close();                   // set thumb to close
  while (!finger[0].reachedPos(100));  // wait for thumb to be fully closed

}

void pinch(void)
{
  int thumbPinchPos = 900;
  int pauseDelay = 1500;
  int pinchSpeed = 220;
  int normalSpeed = 255;

  
  MYSERIAL.println("Close other fingers, leave thumb & index open");
  finger[2].close();                   // set middle to open
  finger[3].close();                   // set ring to open
  finger[4].close();                   // set pinky to open
  while (!finger[2].reachedPos(380));  // wait for middle to be fully open
  while (!finger[3].reachedPos(380));  // wait for ring to be fully open
  while (!finger[4].reachedPos(380));  // wait for pinky to be fully open
  MYSERIAL.println("Fingers closed, moving thumb to position");

  // move thumb into position
  finger[0].writePos(thumbPinchPos);
  while (!finger[0].reachedPos(50));  // wait for thumb to reach position

  MYSERIAL.println("Closing pinch");
  finger[0].writeSpeed(pinchSpeed);
  finger[1].writeSpeed(pinchSpeed);
  
  // close the pinch
  finger[0].close();
  finger[1].close();

   MYSERIAL.println("Closed");

  // wait a bit
  delay(pauseDelay);

   MYSERIAL.println("Opening pinch");

  // open the pinch
  finger[0].writePos(thumbPinchPos);
  finger[1].open();
  while (!finger[0].reachedPos(50));  // wait for thumb to reach position
  while (!finger[1].reachedPos(50));  // wait for index to reach position

  // wait a bit
  delay(pauseDelay);

   MYSERIAL.println("Closing pinch");

  // close the pinch
  finger[0].close();
  finger[1].close();

   MYSERIAL.println("Closed");

  // wait a bit
  delay(pauseDelay);

  MYSERIAL.println("Opening pinch");

  // open the pinch
  finger[0].writePos(thumbPinchPos);
  finger[1].open();
  while (!finger[0].reachedPos(50));  // wait for thumb to reach position
  while (!finger[1].reachedPos(50));  // wait for index to reach position

  finger[0].writeSpeed(normalSpeed);
  finger[1].writeSpeed(normalSpeed);
}

