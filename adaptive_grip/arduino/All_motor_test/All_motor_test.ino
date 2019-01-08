/****************************************************************************** 
SparkFun Big Easy Driver Basic Demo
Toni Klopfenstein @ SparkFun Electronics
February 2015
https://github.com/sparkfun/Big_Easy_Driver

Simple demo sketch to demonstrate how 5 digital pins can drive a bipolar stepper motor,
using the Big Easy Driver (https://www.sparkfun.com/products/12859). Also shows the ability to change
microstep size, and direction of motor movement.

Development environment specifics:
Written in Arduino 1.6.0

This code is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!
Distributed as-is; no warranty is given.

Example based off of demos by Brian Schmalz (designer of the Big Easy Driver).
http://www.schmalzhaus.com/EasyDriver/Examples/EasyDriverExamples.html
******************************************************************************/
//Declare pin functions on Arduino
#define stp 2 //tied to all motors
#define dir 3 //tied to all motors
#define MS1 4 //tied to all motors
#define MS2 5 //tied to all motors
#define MS3 6 //tied to all motors
#define EN_M1 7  //Motor 1 enable
#define EN_M2 8  //Motor 2 enable
#define EN_M3 9  //Motor 3 enable
#define EN_M4 10 //Motor 4 enable
#define EN_M5 11 //Motor 5 enable
#define EN_M6 12 //Motor 6 enable

//Declare variables for functions
char user_input;
int x;
int y;
int state;

void setup() {
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(EN_M1, OUTPUT);
  pinMode(EN_M2, OUTPUT);
  pinMode(EN_M3, OUTPUT);
  pinMode(EN_M4, OUTPUT);
  pinMode(EN_M5, OUTPUT);
  pinMode(EN_M6, OUTPUT);

  resetBEDPins(); //Set step, direction, microstep and enable pins to default states
  Serial.begin(9600); //Open Serial connection for debugging
  Serial.println("Begin motor control");
  Serial.println();
  //Print function list for user selection
  Serial.println("Enter number for control option:");
  Serial.println("1. Motor 1 Forward Default Step Mode");
  Serial.println("2. Motor 1 Reverse Default Step Mode");
  Serial.println("3. Motor 2 Forward Default Step Mode");
  Serial.println("4. Motor 2 Reverse Default Step Mode");
  Serial.println("5. Motor 3 Forward Default Step Mode");
  Serial.println("6. Motor 3 Reverse Default Step Mode");
  Serial.println("7. Motor 4 Forward Default Step Mode");
  Serial.println("8. Motor 4 Reverse Default Step Mode");
  Serial.println("9. Motor 5 Forward Default Step Mode");
  Serial.println("10. Motor 5 Reverse Default Step Mode");
  Serial.println("11. Motor 6 Forward Default Step Mode");
  Serial.println("12. Motor 6 Reverse Default Step Mode");
  Serial.println();
}

//Main loop
void loop() {
  while(Serial.available()){
      user_input = Serial.read(); //Read user input and trigger appropriate function
      //digitalWrite(EN, LOW); //Sample code set all enables to low here, I set them low inside the 'StepForwardDefault'
      if (!user_input%2)
      {
         StepForwardDefault(user_input%2);
      }
      else if(user_input%2)
      {
        StepReverseDefault(user_input%2);
      }
      // else if(user_input =='3')
      // {
      //   SmallStepMode();
      // }
      // else if(user_input =='4')
      // {
      //   ForwardBackwardStep();
      // }
      else
      {
        Serial.println("Invalid option entered.");
      }
      resetBEDPins();
  }
}

//Reset Big Easy Driver pins to default states - no motors are enabled
void resetBEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(EN_M1, LOW);
  digitalWrite(EN_M2, LOW);
  digitalWrite(EN_M3, LOW);
  digitalWrite(EN_M4, LOW);
  digitalWrite(EN_M5, LOW);
  digitalWrite(EN_M6, LOW);
}

//Default microstep mode function
void StepForwardDefault(int mtr)
{
  int mtr_enable = SelectMotorString(floor((mtr+1)/2));
  //Enable selected motor
  digitalWrite(mtr_enable, HIGH);
  Serial.println("Moving forward at default step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  //Disable selected motor
  digitalWrite(mtr_enable,LOW);
  //Prompt for new command
  Serial.println("Enter new option");
  Serial.println();
}


void StepReverseDefault(int mtr)
{
  int mtr_enable =  SelectMotorString(floor((mtr+1)/2));
  //Enable selected motor
  digitalWrite(mtr_enable, HIGH);
  Serial.println("Moving backward at default step mode.");
  digitalWrite(dir, HIGH); //Pull direction pin high to move "backward"
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  //Disable selected motor
  digitalWrite(mtr_enable,LOW);
  //Prompt for new command
  Serial.println("Enter new option");
  Serial.println();
}
//Reverse default microstep mode function
// void ReverseStepDefault()
// {
//   Serial.println("Moving in reverse at default step mode.");
//   digitalWrite(dir, HIGH); //Pull direction pin high to move in "reverse"
//   for(x= 1; x<1000; x++)  //Loop the stepping enough times for motion to be visible
//   {
//     digitalWrite(stp,HIGH); //Trigger one step
//     delay(1);
//     digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
//     delay(1);
//   }
//   Serial.println("Enter new option");
//   Serial.println();
// }

 int SelectMotorString(int mtr){
  if (mtr == 1)
    return EN_M1;
  else if (mtr ==2)
    return EN_M2;
  else if (mtr == 3)
    return EN_M3;
  else if (mtr ==4)
    return EN_M4;
  else if (mtr == 5)
    return EN_M5;
  else if (mtr==6)
    return EN_M6;
  else
    Serial.println("There was an error in the motor number");
  }

// 1/16th microstep foward mode function
// void SmallStepMode()
// {
//   Serial.println("Stepping at 1/16th microstep mode.");
//   digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
//   digitalWrite(MS1, HIGH); //Pull MS1,MS2, and MS3 high to set logic to 1/16th microstep resolution
//   digitalWrite(MS2, HIGH);
//   digitalWrite(MS3, HIGH);
//   for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
//   {
//     digitalWrite(stp,HIGH); //Trigger one step forward
//     delay(1);
//     digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
//     delay(1);
//   }
//   Serial.println("Enter new option");
//   Serial.println();
// }

//Forward/reverse stepping function
// void ForwardBackwardStep()
// {
//   Serial.println("Alternate between stepping forward and reverse.");
//   for(x= 1; x<5; x++)  //Loop the forward stepping enough times for motion to be visible
//   {
//     //Read direction pin state and change it
//     state=digitalRead(dir);
//     if(state == HIGH)
//     {
//       digitalWrite(dir, LOW);
//     }
//     else if(state ==LOW)
//     {
//       digitalWrite(dir,HIGH);
//     }
    
//     for(y=1; y<1000; y++)
//     {
//       digitalWrite(stp,HIGH); //Trigger one step
//       delay(1);
//       digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
//       delay(1);
//     }
//   }
//   Serial.println("Enter new option");
//   Serial.println();
//
