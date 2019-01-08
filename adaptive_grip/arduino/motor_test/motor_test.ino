//Define constants for Enable, Step, Direction, MS1 and MS2 pins

#define EN  2   //Blue
#define MS1 3   //Green
#define MS2 4   //Yellow
#define stp 5   //Orange
#define dir 6   //Purple/White


//Declare variables for functions
char user_input;
int x;
int y;
int state;
int count1;
int count2;
int count3;
int count4;

void setup() {
  // put your setup code here, to run once:
  pinMode(EN,OUTPUT);
  pinMode(stp,OUTPUT);
  pinMode(dir,OUTPUT);
  pinMode(MS1,OUTPUT);
  pinMode(MS2,OUTPUT);

  resetEDPins();
  Serial.begin(9600); //Open Serial connection for debugging
  Serial.println("Begin motor control");
  Serial.println();
  //Print function list for user selection
  Serial.println("Enter number for control option:");
  Serial.println("1. Turn at Full Step.");
  Serial.println("2. Turn at Half Step.");
  Serial.println("3. Turn at Quarter Step .");
  Serial.println("4. Turn at Eighth Step.");
  Serial.println("5. Return to Original Postion.");
  Serial.println();
  count1 = 0;
  count2 = 0;
  count3 = 0;
  count4 = 0;
}

//Main loop
void loop() {
  while(Serial.available()){
      user_input = Serial.read(); //Read user input and trigger appropriate function
      digitalWrite(EN, LOW); //Pull enable pin low to allow motor control
      if (user_input =='1')
      {
         FullStep();
      }
      else if(user_input =='2')
      {
        HalfStep();
      }
      else if(user_input =='3')
      {
        QuarterStep();
      }
      else if(user_input =='4')
      {
        EighthStep();
      }
      else if(user_input =='5')
      {
        OriginalPosition();
        count1 = 0;
        count2 = 0;
        count3 = 0;
        count4 = 0;
      }
      else
      {
        Serial.println("Invalid option entered.");
      }
      resetEDPins();
  }
}

void FullStep()
{
  Serial.println("Moving forward at full step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, LOW); 
  digitalWrite(MS2, LOW);
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
    //Serial.println(count);
  }
  count1+=999;
  Serial.println("Enter new option");
  Serial.println();
}

void HalfStep()
{
  Serial.print("Moving forward at full step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, HIGH); 
  digitalWrite(MS2, LOW);
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
    count2++;
    
  }
  
  Serial.println("Enter new option");
  Serial.println();
}


void QuarterStep()
{
  Serial.println("Moving forward at full step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, LOW); 
  digitalWrite(MS2, HIGH);
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
    count3++;
  }
  Serial.println("Enter new option");
  Serial.println();
}

void EighthStep()
{
  Serial.println("Moving forward at full step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, HIGH); 
  digitalWrite(MS2, HIGH);
  for(x= 1; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
    count4++;
  }
  Serial.println("Enter new option");
  Serial.println();
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
void resetEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(EN, HIGH);
}

