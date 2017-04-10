

// throw up an error if boards are included incorrectly

/* If an error is thrown by one of the statements below, make sure that you have selected the correct board
 *  within 'Tools -> Board' and also make sure that you have uncommented the correct board in MotorTest.ino
 *  
 *  ALMOND_BOARD -> Arduino/Genuino Mega or Mega 2560
 *  CHESTNUT_BOARD -> Chestnut (Open Bionics Boards)
 */
#if defined(ARDUINO_AVR_MEGA2560) && !defined(ALMOND_BOARD)
#error Incorrect board selected, Atmega 2560 is selected in 'Tools -> Board', but ALMOND_BOARD is not selected 
#elif !defined(ARDUINO_AVR_MEGA2560) && defined(ALMOND_BOARD)
#error Incorrect board selected, ALMOND_BOARD is selected, but 'Mega 2560' is not selected in 'Tools -> Board'
#elif defined(ARDUINO_SAMD_CHESTNUT) && !defined(CHESTNUT_BOARD)
#error Incorrect board selected, Chestnut is selected in 'Tools -> Board', but CHESTNUT_BOARD is not selected 
#elif defined(ARDUINO_SAMD_CHESTNUT) && !defined(CHESTNUT_BOARD)
#error Incorrect board selected, CHESTNUT_BOARD is selected, but 'Chestnut' is not selected in 'Tools -> Board'
#endif





// attach the pins to the finger instances
void pinAssignment(void)
{
  if (handFlag == RIGHT)
  {
#if defined(ALMOND_BOARD)
    finger[0].attach(13, 4, A5);     // Right motor connector
    finger[1].attach(3, 6, A1);
    finger[2].attach(7, 8, A2);
    finger[3].attach(10, 9, A3);
    finger[4].attach(11, 12, A4);
#elif defined(CHESTNUT_BOARD)
    finger[0].attach(4, 8, A2, A8, false);  // attach the thumb
    finger[1].attach(1, 2, A0, A6, true);   // attach the index (finger is inverted)
    finger[2].attach(7, 5, A1, A9, true);   // attach the middle (finger is inverted)
    finger[3].attach(0, 9, A3, A7, true);   // attach the ring & pinky (fingers are inverted)
#else
#error 'No board selected'
#endif
  }
  else if (handFlag == LEFT)
  {
#if defined(ALMOND_BOARD)
    finger[0].attach(5, 2, A0);     // Left motor connector
    finger[1].attach(11, 12, A4);
    finger[2].attach(10, 9, A3);
    finger[3].attach(7, 8, A2);
    finger[4].attach(3, 6, A1);
#elif defined(CHESTNUT_BOARD)
    finger[0].attach(7, 5, A1, A9, false);  // attach the thumb 
    finger[1].attach(0, 9, A3, A7, true);   // attach the index (finger is inverted) 
    finger[2].attach(4, 8, A2, A8, true);   // attach the middle (finger is inverted)
    finger[3].attach(1, 2, A0, A6, true);   // attach the ring & pinky (fingers are inverted)  
#else
#error 'No board selected'
#endif
  }
}

