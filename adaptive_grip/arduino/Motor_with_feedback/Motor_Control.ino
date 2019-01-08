void driveMotor(int (*motors), int degRotation, boolean directions[], int (*positions), int arrayLength )
{
  double degPerStp = (double)(1.8 / 8); // This is used for 1/8th stepping mode
  int numSteps = (int)(degRotation / degPerStp);
  boolean dirSwitch = false;
  for (int x = 0; x < numSteps; x++)
  {
    if (dirSwitch) {
      digitalWrite(DIR_1, HIGH);
      digitalWrite(DIR_2, HIGH);
      digitalWrite(DIR_3, HIGH);
      digitalWrite(DIR_4, HIGH);
      digitalWrite(DIR_5, HIGH);
      digitalWrite(DIR_6, HIGH);
      dirSwitch = !dirSwitch;
    }
    else {
      digitalWrite(DIR_1, LOW);
      digitalWrite(DIR_2, LOW);
      digitalWrite(DIR_3, LOW);
      digitalWrite(DIR_4, LOW);
      digitalWrite(DIR_5, LOW);
      digitalWrite(DIR_6, LOW);
      dirSwitch = !dirSwitch;
    }

		// Overwrite directions
    selectMotor(motors, directions, positions);
    delay(2);
    digitalWrite(STP, HIGH);
    delay(2);
    digitalWrite(STP, LOW);
  }
  //debugging
  //  for(int i = 0; i < 6; i++){
  //    Serial.print("Position of motor ");
  //    Serial.print(i+1);
  //    Serial.print(" is: ");
  //    Serial.println(positions[i]);
  //  }
}

void selectMotor(int (*motors), boolean directions[], int (*positions))
{
  for ( int y = 0; y < 6; y++) { //loop through all of the motors
    if (motors[y] != 0) { // mtr[y] set to zero if motor if not intended on moving
      if (directions[y]) { //Open
        if (y == 0 || y == 1 || y == 4) { // These motors close with HIGH direction command
          digitalWrite(motors[y], HIGH);
          positions[y] = positions[y] + 1;
        }
        else {
          if (y == 2 || y == 3 || y == 5) {// These motors close with HIGH directino command
            digitalWrite(motors[y], LOW);
            positions[y] = positions[y] + 1;
          }
          else {
            digitalWrite(motors[y], HIGH);
            Serial.println("Y value is " + String(y));
            Serial.println("ERROR in 'driveMotor', DEBUG");
          }
        }
      }
      else { //Close
        if (y == 0 || y == 1 || y == 4) { // These motors close with LOW direction command
          digitalWrite(motors[y], LOW);
          positions[y] = positions[y] - 1;
        }
        else {
          if (y == 2 || y == 3 || y == 5) {// These motors close with LOW directino command
            digitalWrite(motors[y], HIGH);
            positions[y] = positions[y] - 1;
          }
          else {
            digitalWrite(motors[y], HIGH);
            Serial.println("ERROR in 'driveMotor', wrong y-value selected");
          }
        }
      }
      //debugging
      //Serial.println(positions[y]);

    }
  }
}

void setPins()
{
  digitalWrite(STP, LOW);
  digitalWrite(DIR_1, LOW); // All moving in the same direction
  digitalWrite(DIR_2, LOW);
  digitalWrite(DIR_3, LOW);
  digitalWrite(DIR_4, LOW);
  digitalWrite(DIR_5, LOW);
  digitalWrite(DIR_6, LOW);
}

// Vibrates everthing
// Watch delays
void holdPosition(int mSecs)
{
  for (int x = 0; x < (int)(mSecs / 8); x ++)
  {
    for (int pinOut = DIR_1; pinOut <= DIR_6; pinOut++)
    {
      digitalWrite(pinOut, HIGH);
    }

    digitalWrite(STP, HIGH);
    delay(1);
    digitalWrite(STP, LOW);
    delay(3);

    for (int pinOut = DIR_1; pinOut <= DIR_6; pinOut++)
    {
      digitalWrite(pinOut, LOW);
    }

    digitalWrite(STP, HIGH);
    delay(1);
    digitalWrite(STP, LOW);
    delay(3);
  }
}

void testAllMotors(int (*motors), boolean directions[], int (*positions))
{

  int idx[6] = {0, 0, 0, 0, 0, 0};

  for (idx[0] = 0; idx[0] < 2; idx[0]++) {
    for (idx[1] = 0; idx[1] < 2; idx[1]++) {
      for (idx[2] = 0; idx[2] < 2; idx[2]++) {
        for (idx[3] = 0; idx[3] < 2; idx[3]++) {
          for (idx[4] = 0; idx[4] < 2; idx[4]++) {
            for (idx[5] = 0; idx[5] < 2; idx[5]++) {
              zeroIntArray(motors, 6);
              falseBooleanArray(directions, 6);
              setMotors(idx, motors);
              driveMotor(motors, 45, directions, positions, 6);
            }
          }
        }
      }
    }
  }
}

void setMotors(int (*index), int (*motors))
{
  for (int x = 0; x < 6; x++) {
    if (index[x] == 1)
      motors[x] = x + 3; //DIR_1 is three, DIR_5 is 7;
    else
      motors[x] = 0;
  }
}

void setOriginalPosition(int enablePin, int (*positions))
{
  char userInput;
  Serial.println("Please turn the motors to their initial positions, then enter some serial data");
  digitalWrite(enablePin, HIGH);
  while (!Serial.available()) { //Allows the motors to not step - no holding torque
  }
  for (int x = 0; x < 6; x++) {
    positions[x] = 0; //resets the position
  }
  digitalWrite(enablePin, LOW); //Turns the motors back on - draws max current so don't remain in this state for a long time
}

void openFinger1(int (*motors), boolean directions[], int (*positions)) 
{
  setPins();
  directions[1] = true;
  directions[3] = true;
  motors[1] = DIR_2; // these two motors will be moving
  motors[3] = DIR_4;
  int limit_1 = 900;
  int limit_3 = 630;

  while (positions[1] < limit_1 || positions[3] < limit_3) {
    if (positions[1] < limit_1 && positions[3] < limit_3) {
      motors[1] = DIR_2; // these two motors will be moving
      motors[3] = DIR_4;
    }
    else {
      if (positions[1] < limit_1)
      {
        motors[1] = DIR_2; // these two motors will be moving
        motors[3] = 0;
      }
      else {
        motors[1] = 0; // these two motors will be moving
        motors[3] = DIR_4;
      }
    }
    driveMotor(motors, 5, directions, positions, 6);
  }
}
void openFinger3(int (*motors), boolean directions[], int (*positions)) 
{
  setPins();
  directions[2] = true;
  directions[4] = true;
  int limit_1 = 900;
  int limit_3 = 600;

  while (positions[2] < limit_1 || positions[4] < limit_3) {
    if (positions[2] < limit_1 && positions[4] < limit_3) {
      motors[2] = DIR_3; // these two motors will be moving
      motors[4] = DIR_5;
    }
    else {
      if (positions[2] < limit_1)
      {
        motors[2] = DIR_3; // these two motors will be moving
        motors[4] = 0;
      }
      else {
        motors[2] = 0; // these two motors will be moving
        motors[4] = DIR_5;
      }
    }
    driveMotor(motors, 5, directions, positions, 6);
  }
}

void openAllJoints(int (*motors), boolean directions[], int (*positions), int(*limits))
{
  for (int i = 0; i < 6; i++)
  {
    directions[i] = true;
  }

  int check = 1;
  while(check != 0)
  {
    check = 0;
    for (int i = 0; i < 6; i++)
      {

        if(positions[i] < limits[i])
        {
          motors[i] = i+3;
          check = check + 1;
        }
        else // (positionspi] >= limits[i])
          motors[i] = 0;
      }
   if (check != 0)
    driveMotor(motors,5,directions,positions,6);
  }
}

void closeAllJoints(int (*motors), boolean directions[], int (*positions))
{
  for (int i = 0; i < 6; i++)
  {
    directions[i] = false;
  }

  int check = 1;
  while(check != 0)
  {
    check = 0;
    for (int i = 0; i < 6; i++)
      { 
        if(positions[i] > 20) //leave a little room so that i don't over extend
        {
          motors[i] = i+3;
          check = check + 1;
        }
        else // (positionspi] >= limits[i])
          motors[i] = 0;
      }
   if (check != 0)
    driveMotor(motors,5,directions,positions,6);
  }
}
//added force parameter
void curlGrab(int(*motors),boolean directions[], int(*positions), int (*limits), int *(force))
{
  checkPressure(force);

  for (int i = 0; i < 6; i++)
    directions[i] = false;

  int check = 1;
  while(check != 0)
  {
    //Serial.println(force[0]);
    //Serial.println(force[1]);
    //Serial.println(force[2]);
    check = 0;
    for (int i = 0; i < 6; i++)
       check = motorOperation(i, check, motors, force, limits, positions);
    if (check != 0)
      driveMotor(motors,5,directions,positions,6);
  }
  

}

