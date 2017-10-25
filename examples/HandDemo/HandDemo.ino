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
//#define ALMOND_BOARD
#define CHESTNUT_BOARD

// number of controllable fingers (number of motors)
#if defined(ALMOND_BOARD)
#define NUM_FINGERS 5
#define MYSERIAL Serial
#elif defined(CHESTNUT_BOARD)
#define NUM_FINGERS 4
#define MYSERIAL SerialUSB
#else
#error 'No board selected'
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


// rolls the fingers closed and open, the next finger starts moving when the previous finger reaches it's target position
void fingerRoll(void)
{
  const int delTime = 400;
  int i;
  MYSERIAL.println("Fingers rolling closed");
  //for (i = 0; i < NUM_FINGERS; i++)
  for (i = NUM_FINGERS-1; i >= 0; i--)
  {
    finger[i].close();    // set finger to close
    delay(delTime);       // allow time for fingers to close
  }
  MYSERIAL.println("Fingers rolling open");
  //for (i = 0; i < NUM_FINGERS; i++)
  for (i = NUM_FINGERS-1; i >= 0; i--)
  {
    finger[i].open();     // set finger to open
    delay(delTime);       // allow time for fingers to open
  }
}

// the next finger starts moving when the previous finger reaches it's target position
void thumbsUp(void)
{
  const int delTime = 400;
  
  MYSERIAL.println("Close all fingers");
  closeHand();
  delay(delTime);       // allow time for fingers to close

  MYSERIAL.println("Wait for thumb to open");
  finger[0].open();     // set thumb to open
  delay(delTime);       // allow time for thumb to open

  MYSERIAL.println("Wait 1s with thumb open");
  delay(1000);        // wait for a bit

  MYSERIAL.println("Close thumb");
  finger[0].close();    // set thumb to close
  delay(delTime);       // allow time for thumb to close

}

void pinch(void)
{
  const int delTime = 400;
  const int pauseDelay = 1500;
  const int thumbPinchPos = 900;
  const int pinchSpeed = 220;
  const int normalSpeed = 255;

  
  MYSERIAL.println("Close other fingers, leave thumb & index open");
  finger[2].close();                   // set middle to open
  finger[3].close();                   // set ring to open
  finger[4].close();                   // set pinky to open
  delay(delTime);                      // allow time for fingers to close
  MYSERIAL.println("Fingers closed, moving thumb to position");

  finger[0].writePos(thumbPinchPos);  // move thumb into position
  delay(delTime);                     // allow time for thumb to close

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
  delay(delTime);                      // allow time for fingers to open

  // wait a bit
  delay(pauseDelay);

  // close the pinch
  MYSERIAL.println("Closing pinch");
  finger[0].close();
  finger[1].close();

  // wait a bit
  delay(pauseDelay);
  MYSERIAL.println("Closed");

  
  MYSERIAL.println("Opening pinch");

  // open the pinch
  finger[0].writePos(thumbPinchPos);
  finger[1].open();
  delay(delTime);                      // allow time for fingers to open

  finger[0].writeSpeed(normalSpeed);
  finger[1].writeSpeed(normalSpeed);
}
