/*  Open Bionics - FingerLib Example - SingleFinger
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
 * Uses the FingerLib.h library to open and close a single finger every 2 seconds
 * 
 */

// initialise Finger class to array, and assigns pins
Finger finger;

void setup()
{ 
  // MYSERIAL.print is used to allow compatability between both the Mega (Serial.print) 
  //and the Zero Native Port (SerialUSB.print), and is defined in FingerLib.h
  MYSERIAL.begin(38400);
  MYSERIAL.println("Started");

  // configure finger pins
  finger.attach(3,6,A1);
  MYSERIAL.println("Pins configured");
}

void loop()
{
  // set the finger to OPEN
  finger.open();
  MYSERIAL.println("Opening");
  // give it time to OPEN
  delay(2000);
  // set the finger to CLOSE
  finger.close();
  MYSERIAL.println("Closing");
  // give it time to CLOSE
  delay(2000);

}


