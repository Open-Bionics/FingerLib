/*  Open Bionics - FingerLib Example - MotorTest
  Author - Olly McBride
  Date - October 2015

  This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
  To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/.

  Website - http://www.openbionics.com/
  GitHub - https://github.com/Open-Bionics
*/

#include <FingerLib.h>

/*
   WHAT IT DOES
   Reads the current position of each finger and prints the ADC values over Serial
   Motors can be controlled/toggled (open/close) by entering the corresponding motor number (0 - 5)
   Entering the number equivalent to the number of motors toggles all of the motors at once (6)
   Entering any value greater than the number of motors stops all of the motors (>6)

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


// MACROS
#define IS_BETWEEN(x,a,b) ((((x)>=(a))&&((x)<=(b)))?(1):(0))        // check if value x is between values a and b

// initialise Finger class to array
Finger finger[NUM_FINGERS];

char* names[MAX_FINGERS] = {" m0", " m1", " m2", " m3", " m4", " m5"}; // motor names
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
  char rxChar;

  // if a serial char is available
  if (MYSERIAL.available())
  {
    rxChar = MYSERIAL.read();

    // if char is an EOL char, ignore it
    if ((rxChar != '\n') && (rxChar != '\r'))
    {
      fingerNum = (int) rxChar - 48;     // convert ASCII to number

      if (IS_BETWEEN(fingerNum, 0, (NUM_FINGERS - 1))) // if the number entered is a valid finger number
      {
        finger[fingerNum].open_close();           // toggle finger direction
      }
      else if (fingerNum == NUM_FINGERS)          // else if the number entered is equal to the number of fingers
      {
        for (int i = 0; i < NUM_FINGERS; i++)
        {
          finger[i].open_close();                 // toggle direction of all fingers
        }
      }
      else                                        // else if another character is entered
      {
        stopMotors();                             // stop all motors
        MYSERIAL.println("MOTORS STOPPED");
        delay(500);
      }
    }
  }

  // print current finger position
  for (i = 0; i < NUM_FINGERS; i++)
  {
    MYSERIAL.print(names[i]);
    MYSERIAL.print(" = ");
    MYSERIAL.print(finger[i].readPos());
    MYSERIAL.print("\t");
    delay(10);
  }
  MYSERIAL.println();

}

// stop all motors and maintain the current position
void stopMotors(void)
{
  int i;
  for (i = 0; i < NUM_FINGERS; i++)
  {
    finger[i].stopMotor();
  }
}
