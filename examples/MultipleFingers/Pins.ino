

void pinAssignment(void)
{
  if(handFlag == RIGHT)
  {
    #if defined(ADULT_BOARD)
      finger[0].attach(8,9,A4);
      finger[1].attach(7,6,A3);
      finger[2].attach(5,12,A2);
      finger[3].attach(2,10,A1);
      finger[4].attach(4,3,A0);
    #elif defined(FOUR_MOTOR_BOARD)
      finger[0].attach(2,10,A1);  
      finger[1].attach(7,6,A3);
      finger[2].attach(5,12,A2);
      finger[3].attach(4,3,A0);
    #elif defined(CHILD_BOARD)
      finger[0].attach(2,10,A1);
      finger[1].attach(8,9,A4);
      finger[2].attach(4,3,A0);
    #elif defined(ALMOND_BOARD)
      finger[0].attach(5,2,A0);
      finger[1].attach(3,6,A1);
      finger[2].attach(7,8,A2);
      finger[3].attach(10,9,A3);
      finger[4].attach(11,12,A4);
    #else
      #error 'No board selected'
    #endif
  }
  else if(handFlag == LEFT)
  {
    #if defined(ADULT_BOARD)
      finger[0].attach(8,9,A4);
      finger[4].attach(4,3,A0);
      finger[3].attach(2,10,A1);
      finger[2].attach(5,12,A2);
      finger[1].attach(7,6,A3);
    #elif defined(FOUR_MOTOR_BOARD)
      finger[0].attach(5,12,A2);
      finger[1].attach(4,3,A0);
      finger[2].attach(2,10,A1); 
      finger[3].attach(7,6,A3);
    #elif defined(CHILD_BOARD)
      finger[0].attach(2,10,A1);
      finger[1].attach(4,3,A0);
      finger[2].attach(8,9,A4);
    #elif defined(ALMOND_BOARD)
      finger[0].attach(5,2,A0);
      finger[1].attach(11,12,A4);
      finger[2].attach(10,9,A3);
      finger[3].attach(7,8,A2);
      finger[4].attach(3,6,A1);
    #else
      #error 'No board selected'
    #endif
  }
}

