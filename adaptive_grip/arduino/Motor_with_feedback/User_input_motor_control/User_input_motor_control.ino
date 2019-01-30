
// Pin defs
#define STP 2 // All motor steps are tied together
#define DIR_1 3 // Each motor has its own direction pin
#define DIR_2 4
#define DIR_3 5
#define DIR_4 6
#define DIR_5 7
#define DIR_6 8
#define EN 10
#define ARRAY_SIZE 6
#define THRESHOLD_INCREASE 68
//int forceThreshold[6] = {562+40,250+40,470+40,193+40,455+40,438+40};
//added new array all initialized to zero
int forceThreshold[6] = {0,0,0,0,0,0};
//Declare variables for functions
char userInput;
boolean firstRun = true;
//Arrays
int     motors[ARRAY_SIZE] = {0,0,0,0,0,0}; //rotate motor or vibrate in place
// True = open
// False = close
boolean   directions[ARRAY_SIZE] = {false,false,false,true,true,true};//true is open, false is close
// Counter to remember how many steps each motor's moved
int     positions[ARRAY_SIZE] = {0,0,0,0,0,0}; //with respect to 'home' position
// Bounds of the steps - 0 is the lower limit for all
//int   homeLimits[ARRAY_SIZE] = {0,0,0,0,0,0};
int   outerLimits[ARRAY_SIZE] = {900,900,900,600,600,650};
int   innerLimits[ARRAY_SIZE] = {0,0,0,0,0,0}; //This was previously {400, 400, 400, 400, 400 ,400, 400, 400}
// "Curl Limits"
int   curlLimits[ARRAY_SIZE] =  {400,400,400,50,50,50};
int   clampLimits[ARRAY_SIZE] = {400,400,400,450,450,450};
int force[6] = {0,0,0,0,0,0};
//force[0]: paired with motor 1 and 6, A0
//force[1]: paired with motor 2 and 4, A2
//force[2]: paired with motor 3 and 5. A1
//force[3]: paired with motor 1 and 6, A3
//force[4]: paired with motor 2 and 4, A5
//force[3]: paired with motor 3 and 5, A4
// function declarations
void printArrays(int(*motors), boolean directions[], int arrayLength);
void zeroIntArray(int (*motors), int arrayLength);
void falseBooleanArray(boolean directions[], int arrayLength);
void printPositions(int (*positions));
void userMotorChoice(char userInput, int (*motors), boolean directions[], int (*force), int (*innerLimits), int (*outerLimits), int (*forceThreshold));
void displayMenu();
void driveMotor(int (*motors), int degRotation, boolean directions[], int (*positions), int arrayLength );
void selectMotor(int (*motors), boolean directions[], int (*positions));
void setPins();
void holdPosition(int mSecs);
void testAllMotors(int (*motors), boolean directions[], int (*positions));
void setMotors(int (*index), int (*motors));
void setOriginalPosition(int enablePin, int (*positions));
void openFinger1(int (*motors), boolean directions[], int (*positions));
void openFinger3(int (*motors), boolean directions[], int (*positions));
void openAllJoints(int (*motors), boolean directions[], int (*positions), int(*limits));
void closeAllJoints(int (*motors), boolean directions[], int (*positions), int (*innerLimits));
void curlGrab(int(*motors),boolean directions[], int(*positions), int (*innerLimits), int (*outerLimits));
void checkPressure(int (*force));
boolean curlGrab(int(*motors), boolean directions[], int(*positions), int (*innerLimits), int (*force), int (*forceThreshold));

void setup() { //--------------------------------------------------- SETUP -----------------------------------------------------------------//
pinMode(STP,OUTPUT);
pinMode(DIR_1,OUTPUT);
pinMode(DIR_2,OUTPUT);
pinMode(DIR_3,OUTPUT);
pinMode(DIR_4,OUTPUT);
pinMode(DIR_5,OUTPUT);
pinMode(DIR_6,OUTPUT);
pinMode(EN,OUTPUT);
pinMode(A0, INPUT);
pinMode(A1, INPUT);
pinMode(A2, INPUT);
pinMode(A3, INPUT);
pinMode(A4, INPUT);
pinMode(A5, INPUT);
setPins(); 
  Serial.begin(9600); 
displayMenu();
}

void loop() { // ------------------------------------------------------- MAIN LOOP --------------------------------------------------//
if (firstRun){
setOriginalPosition(EN, positions); //home the robot manually, then enter input through serial monitor to 'lock' robot and store 'zero' positions
    firstRun = false;  
  }
while (Serial.available()) {
    userInput = Serial.read();
if (userInput =='0'){
printPositions(positions);
displayMenu();
    }
else{
zeroIntArray(motors,ARRAY_SIZE); //sets all motors to 'do not move' by setting each value of the array to '0'
userMotorChoice(userInput, motors, directions, force, innerLimits, outerLimits, forceThreshold); // Sets new motors array and directions array
driveMotor(motors, 5, directions, positions, ARRAY_SIZE); 
    } 
  }
}

// ------------------------------------------------------------------- FUNCTIONS ----------------------------------------------------//
void driveMotor(int (*motors), int degRotation, boolean directions[], int (*positions), int arrayLength )
{
  double degPerStp = (double)(1.8 / 8); // This is used for 1/8th stepping mode
  int numSteps = (int)(degRotation / degPerStp);
  boolean dirSwitch = false;
  for (int x = 0; x < numSteps; x++)
  {
    if (dirSwitch) { //Sets the motor to open
      digitalWrite(DIR_1, HIGH);
      digitalWrite(DIR_2, HIGH);
      digitalWrite(DIR_3, HIGH);
      digitalWrite(DIR_4, HIGH);
      digitalWrite(DIR_5, HIGH);
      digitalWrite(DIR_6, HIGH);
      dirSwitch = !dirSwitch;
    }
    else { //Sets the motor to close
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

void testAllMotors(int (*motors), boolean directions[], int (*positions)) //test code FOR MOTORS ONLY, do not use when the motors are attached to the robot
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


void openAllJoints(int (*motors), boolean directions[], int (*positions), int(*outerLimits))
{
  for (int i = 0; i < 6; i++)
  {
    if (positions[i] < outerLimits[i])
      directions[i] = true; // open
    else
      directions[i] = false; //close
  }

  int check = 1;
  while (check != 0)
  {
    check = 0;
    for (int i = 0; i < 6; i++)
    {
      if (true) //motor 5 does not work
      {

       // if (i == 4) continue;

        if (positions[i] < outerLimits[i])
        {
          motors[i] = i + 3;
          check = check + 1;
        }
        else // (positionspi] >= outerLimits[i])
          motors[i] = 0;
      }
      else // (positionspi] >= outerLimits[i])
        motors[i] = 0;
    }
    if (check != 0)
      driveMotor(motors, 5, directions, positions, 6);
  }
}

void closeAllJoints(int (*motors), boolean directions[], int (*positions), int (*innerLimits))
{
  for (int i = 0; i < 6; i++)
  {
   // if (i == 4) continue;

    directions[i] = false;
  }

  int check = 1;
  while (check != 0)
  {
    check = 0;
    for (int i = 0; i < 6; i++)
    {
      if (positions[i] > innerLimits[i] + 20) //leave a little room so that i don't over extend
      {
        motors[i] = i + 3;
        check = check + 1;
      }
      else // (positionspi] >= limits[i])
        motors[i] = 0;
    }
    if (check != 0)
      driveMotor(motors, 5, directions, positions, 6);
  }
}

 boolean curlGrab(int(*motors), boolean directions[], int(*positions), int (*clampLimits), int (*force), int (*forceThreshold))
{
  boolean objectWasGrabbed = false;
  for (int i = 0; i < 6; i++)
  {
    directions[i] = false;
  }
  int check = 1;

  //added checkPressure
  checkPressure(force);
  //added for loop; forceThreshold values are 40 above intial force values
  for(int i = 0; i<6; i++){
    forceThreshold[i] = force[i] + THRESHOLD_INCREASE;
  }
  while (check != 0)
  {
    check = 0;
    for (int i = 0; i < 6; i++)
    {
      if (positions[i] > clampLimits[i])
      {
        motors[i] = i + 3;
        check = check + 1;
      }
      else // (positionspi] <= limits[i])
      {
        motors[i] = 0;
      }

      checkPressure(force);
//      for (int i = 0; i < 6; i++)
//      {
//        Serial.print(" Sensor  ");
//        Serial.print(i);
//        Serial.print(" is : ");
//        Serial.print(force[i]);
//      }
      //Serial.println(" ");
      if (force[i] < forceThreshold[i])
      { // No problems
      }
      else  // Have reached the threshold
      {
        objectWasGrabbed = true;
        for (int i = 0; i < 6; i ++)
        {
          motors[i] = 0;
        }
        return objectWasGrabbed;
      }

    }
    if (check > 0)
        driveMotor(motors, 5, directions, positions, 6);
  }
  return objectWasGrabbed;
}

void printArrays(int(*motors), boolean directions[], int arrayLength){
  for (int x = 0; x < arrayLength; x++){
    Serial.print("motors[" + String(x) + "] = " + String(motors[x]) + " | ");
  }
  Serial.println();
  for (int x = 0; x < arrayLength; x++){
    Serial.print("directions[" + String(x) + "] = " + String(directions[x]) + " | ");
  }
  Serial.println();
}

// -----------------------------------Reset Array ----------------------------//
void zeroIntArray(int (*motors), int arrayLength)
{
  for (int x = 0; x < arrayLength; x ++){
    motors[x] = 0;
  } 
}

//------------------------------------Reset Boolean Array --------------------//
void falseBooleanArray(boolean directions[], int arrayLength)
{
  for (int x = 0; x < arrayLength; x++){
    directions[x] = false;
  }
}

void printPositions(int (*positions)){
  for (int x = 0; x < 6; x++){
    Serial.print("position[" + String(x) + "] :" + String(positions[x]) + "   |   ");
  }
  Serial.println("");
}

void userMotorChoice(char userInput, int (*motors), boolean directions[], int (*force), int (*innerLimits), int (*outerLimits), int (*forceThreshold))
{
  switch (userInput) {
    case '1': {
        motors[0] = DIR_1;
        directions[0] = true;
        break;
    } case '2': {
        motors[1] = DIR_2;
        directions[1] = true;
        break;
    } case '3': {
        motors[2] = DIR_3;
        directions[2] = true;
        break;
    } case '4': {
        motors[3] = DIR_4;
        directions[3] = true;
        break;
    } case '5': {
        motors[4] = DIR_5;
        directions[4] = true;
        break;
    } case '6': {
        motors[5] = DIR_6;
        directions[5] = true;
        break;
    } case '7': {
        motors[0] = DIR_1;
        directions[0] = false;
        break;
    } case '8': {
        motors[1] = DIR_2;
        directions[1] = false;
        break;
    } case 'a': {
        motors[2] = DIR_3;
        directions[2] = false;
        break;
    } case 'b': {
        motors[3] = DIR_4;
        directions[3] = false;
        break;
    } case 'c': {
        motors[4] = DIR_5;
        directions[4] = false;
        break;
    } case 'd': {
        motors[5] = DIR_6;
        directions[5] = false;
        break;
    } case 'e': {
        openAllJoints(motors,directions,positions,outerLimits);
        curlGrab(motors, directions, positions, clampLimits, force, forceThreshold);
        break;
    } case 'f':{
        closeAllJoints(motors,directions,positions,innerLimits);
        break;
    } case 'h': {
        openAllJoints(motors,directions,positions,outerLimits);
        break;
    }
    case 'i': {
        openAllJoints(motors,directions,positions,outerLimits);
        curlGrab(motors,directions,positions,innerLimits,force, forceThreshold);
        break;
    }

    case 'z': {
        firstRun = true;
        setPins();
        break;
    }
      
    default: {
        //do something
        Serial.println("ERROR, User input was incorrect");
        directions[0] = false;
        break;
      }
  }
}



void displayMenu(){
  Serial.println("Enter number for control option:");
  Serial.println("0. Print Positions");
  Serial.println("1. Motor 1, Clockwise");
  Serial.println("2. Motor 2, Clockwise");
  Serial.println("3. Motor 3, Clockwise");
  Serial.println("4. Motor 4, Clockwise");
  Serial.println("4. Motor 4, Clockwise");
  Serial.println("5. Motor 5, Clockwise");
  Serial.println("6. Motor 6, Clockwise");
  Serial.println("7. Motor 1, Counter-Clockwise");
  Serial.println("8. Motor 2, Counter-Clockwise");
  Serial.println("a. Motor 3, Counter-Clockwise");
  Serial.println("b. Motor 4, Counter-Clockwise");
  Serial.println("c. Motor 5, Counter-Clockwise");
  Serial.println("d. Motor 6, Counter-Clockwise");
  Serial.println("e. Curl Grab");
  Serial.println("z. Reset positions, claw limp");
  Serial.println("f. Close All Joints");
  Serial.println("h. Open All Joints");
  Serial.println();
}



//------------------------------- FEEDBACK ------------------------------------------------------------//

/**
 * function: checkPressure - reads the analog input at A0, A2, A3, which detail the amount of force being replied to the sensors
 * parameters: *force - integer array pointer of the force values
 */


void checkPressure(int (*force)){
  force[0] = analogRead(A2);//1-6
  force[1] = analogRead(A0);//2-4
  force[2] = analogRead(A1);//3-5
}
