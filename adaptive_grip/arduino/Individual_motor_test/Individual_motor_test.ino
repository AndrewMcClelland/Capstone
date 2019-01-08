//Define constants for Enable, Step, Direction, MS1 and MS2 pins

//Available digital pins are: 0,1,2,13
#define EN_1 7
#define EN_2 8
#define EN_3 9
#define EN_4 10
#define EN_5 11
#define EN_6 12
#define MS1 3   //Green
#define MS2 4   //Yellow
#define stp 5   //Orange
#define dir 6   //Purple/White


//Declare variables for functions
//char userInput;
int desiredDegrees;
int x;
int y;
int state;
int count1;
int count2;
int count3;
int count4;
boolean changeD;

void setup() {
  // put your setup code here, to run once:
  pinMode(EN_1,OUTPUT);
  pinMode(EN_2,OUTPUT);
  pinMode(EN_3,OUTPUT);
  pinMode(EN_4,OUTPUT);
  pinMode(EN_5,OUTPUT);
  pinMode(EN_6,OUTPUT);
  pinMode(stp,OUTPUT);
  pinMode(dir,OUTPUT);
  pinMode(MS1,OUTPUT);
  pinMode(MS2,OUTPUT);

  resetEDPins();
  Serial.begin(9600); //Open Serial connection for debugging
  //Print function list for user selection
  Serial.println("Ready to Test?");
  
  
  Serial.println();
  count1 = 0;
  count2 = 0;
  count3 = 0;
  count4 = 0;
  changeD = false;
}

//Main loop
void loop() {
      
      //userInput = Serial.read(); //Read user input and trigger appropriate function 
      /*     
      String input = String(1);
      controlMotor((int)(6+1));
      Serial.println("Moving motor number " + input);
      desiredDegrees = 720;
      driveMotor(desiredDegrees,1,HIGH);
      resetEDPins();
      while(1){};
      */
      /*
      controlMotor(1+6);
      driveMotor(360,1,HIGH);
      //resetEDPins();

      controlMotor(2+6);
      driveMotor(360,1,HIGH);
      //resetEDPins();
      
      controlMotor(3+6);
      driveMotor(360,1,HIGH);
      //resetEDPins();
      
      controlMotor(4+6);
      driveMotor(360,1,HIGH);
      //resetEDPins();
      
      controlMotor(5+6);
      driveMotor(360,1,HIGH);
      //resetEDPins();
      
      controlMotor(6+6);
      driveMotor(360,1,HIGH);
      //resetEDPins();
      */
      //digitalWrite(EN_1, LOW);
      digitalWrite(EN_2, LOW);
      //digitalWrite(EN_3, LOW);
      //digitalWrite(EN_4, LOW);
      //digitalWrite(EN_5, LOW);
      //digitalWrite(EN_6, LOW);
      for(x= 0; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
      {/*
        if(changeD){
          changeD = false;
          digitalWrite(dir, LOW);
        }
        else{
          changeD = true;
          digitalWrite(dir, HIGH);
        }*/
        digitalWrite(stp,HIGH); //Trigger one step forward
        delay(2);
        digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
        delay(2);
      }
      resetEDPins();

      //holdPosition();
      //delay(5000);
      //resetEDPins();  
}






void OriginalPosition()
{
  Serial.println("Move back to origianl position");
  digitalWrite(dir, HIGH); //Pull direction pin high to move "forward"
  
  digitalWrite(MS1, LOW); 
  digitalWrite(MS2, LOW);
  for(x= 1; x< count1; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }

  digitalWrite(MS1, HIGH); 
  digitalWrite(MS2, LOW);
  for(x= 1; x< count2; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }

  digitalWrite(MS1, LOW); 
  digitalWrite(MS2, HIGH);
  for(x= 1; x< count3; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }

  digitalWrite(MS1, HIGH); 
  digitalWrite(MS2, HIGH);
  for(x= 1; x< count4; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println("Enter new option");
  Serial.println();
}



//Reset Easy Driver pins to default states


