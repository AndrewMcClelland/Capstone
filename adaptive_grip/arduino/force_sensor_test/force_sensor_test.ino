//Resistance increases as the pressure increases
//no pressure : analogRead() = 0 
//as pressure increases, analogRead() value increases
// recommended threshold of at least 
// may need to make it dynamic based on type of object

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensor0;
  int sensor1;
  int sensor2;

  sensor0 = analogRead(A0);
  sensor1 = analogRead(A1);
  sensor2 = analogRead(A2);

  Serial.print("Value of Sensor 0: ");
  Serial.print(sensor0);
  Serial.print("                    ");

  Serial.print("Value of Sensor 1: ");
  Serial.print(sensor1);
  Serial.print("                    ");

  Serial.print("Value of Sensor 2: ");
  Serial.println(sensor2);

  delay(500);
  

}
