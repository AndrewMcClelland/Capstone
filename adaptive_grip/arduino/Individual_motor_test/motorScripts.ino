/**
 * function: controlMotor
 * Enables one of the 6 gripper motors to turn by setting its enable pin to "LOW"
 * 
 * motorID: integer ID of the motor
 */

void controlMotor(int motorID)
{
  switch(motorID)
  {
    //Serial.println("Moving Motor Number" + motorID);
    case EN_1: //(7)NEMA23
    {
      digitalWrite(EN_1,LOW);
      break;
    }
    case EN_2: //(8)NEMA23
    {
      digitalWrite(EN_2,LOW);
      break;
    }
    case EN_3: //(9)NEMA23
    {
      digitalWrite(EN_3,LOW);
      break;
    }
    case EN_4: //(10)NEMA17
    {
      digitalWrite(EN_4,LOW);
      break;
    }
    case EN_5: //(11)NEMA17
    {
      digitalWrite(EN_5,LOW);
      break;
    }
    case EN_6: //(12)NEMA17
    {
      digitalWrite(EN_6,LOW);
      break;
    }
  }
  
}

/**
 * Function: driveMotor
 * 
 * Rotates the motor(s) by a specified number of degrees at full, half, quarter or eight step speed in
 * either clockwise or counter-clockwise direction
 * 
 * degrees: amount by which the motor is rotated
 * stepStyle: absolute change in angular displacement motor takes with each 'step'
 * highOrLow: the direction that the motor will rotate; CW vs CCW
 */

void driveMotor(int degrees, int stepStyle, uint8_t highOrLow )
{
  int rotate = 0;
  digitalWrite(dir,highOrLow);
switch(stepStyle){
  case 0: { // Full Step
    Serial.println("Moving at full step mode.");
    digitalWrite(MS1, LOW); 
    digitalWrite(MS2, LOW);
    rotate = (int)(degrees/1.8);
    break;
  }
  case 1: { // Half Step
    Serial.println("Moving at half step mode.");
    digitalWrite(MS1, HIGH); 
    digitalWrite(MS2, LOW);
    rotate = 2*(int)(degrees/1.8);
    break;
  }
  case 2: { // Quarter Step
    Serial.println("Moving at quarter step mode.");
    digitalWrite(MS1, LOW); 
    digitalWrite(MS2, HIGH);
    rotate = 4*(int)(degrees/1.8);
    break;
  }
  default: { // Eighth Step
    Serial.println("Moving at eigth step mode.");
    digitalWrite(MS1, HIGH); 
    digitalWrite(MS2, HIGH);
    rotate = 8*(int)(degrees/1.8);
    break;
  }
}
 
  for(x= 1; x<rotate; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
    count2++;
  }
}


/**
 * Function: Reset pins
 * 
 * Disables all 6 motors from moving.  Changes motor step size to full step.
 */
void resetEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(EN_1, HIGH);
  digitalWrite(EN_2, HIGH);
  digitalWrite(EN_3, HIGH);
  digitalWrite(EN_4, HIGH);
  digitalWrite(EN_5, HIGH);
  digitalWrite(EN_6, HIGH);
}

void holdPosition()
{
  digitalWrite(EN_1, LOW);
  digitalWrite(EN_2, LOW);
  digitalWrite(EN_3, LOW);
  //digitalWrite(EN_4, LOW);
  //digitalWrite(EN_5, LOW);
  //digitalWrite(EN_6, LOW);
}
