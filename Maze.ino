#include <Arduino.h>

short motorPower = 100;
short straightConstant = 1090; //1090
short turnConstant = 590; //620 previously

constexpr byte leftEncoderChannelA = 19;      //Left Encoder Channels
constexpr byte leftEncoderChannelB = 18;
constexpr byte rightEncoderChannelA = 20;      //Right Encoder Channels
constexpr byte rightEncoderChannelB = 21;


const int leftMotorChannelA = 9;             //Left Motor Pins
const int leftMotorChannelB = 8;
const int rightMotorChannelA = 10;             //Right Motor Pins
const int rightMotorChannelB = 11;

volatile long leftPosition = 0;               //Left Encoder Position Variables
volatile long rightPosition = 0;              //Right Encoder Position Variables

float leftUp = 0;                             //Left Motor PID Variables
float leftKp = 1.08;
float leftProportional = 0;
float leftKd = 45;
float leftDerivative = 0;
long leftError = 0;
long leftPrevError = 0;
unsigned long leftPrevTime = 0;

float rightUp = 0;                            //Right Motor PID Variables
float rightKp = 2.2; 
float rightProportional = 0;
float rightKd = 1;
float rightDerivative = 0;
long rightError = 0;
long rightPrevError = 0;
unsigned long rightPrevTime = 0;

unsigned long startTime;
unsigned long timer;
long timeout = 1500;
float maxError = 1.05;
float minError = 0.95;


const byte frontTrig = 48;                     //Ultrasonic Trig and Echo Pins
const byte frontEcho = 49;
const byte leftTrig = 50;
const byte leftEcho = 51;
const byte rightTrig = 52;
const byte rightEcho = 53;
const byte frontBottomTrig = 46; 
const byte frontBottomEcho = 47;

long distanceCheck;                           //Ultrasonic Distance Variables
long frontDistance = 101;
long leftDistance = 101;
long rightDistance = 101;
long frontBottomDistance = 101;

float levelKp = 0.1;                          //Distance PID Variables
float levelError = 0;
float levelProportional = 0;

unsigned long lastPingTime = 0;               //Ping Timings
const unsigned long pingInterval = 50;


bool mazeMap[6][4] = 
{
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 1, 0}
};

short mazeMapRow = 5;
short mazeMapColumn = 2;
short direction = 0;
int cellCheck = 0;
bool cellState = 0;
unsigned long cellCheckTimer = 0;

short route[24] = {0};
int routePos = -1;

//Function to run motors
void Control(int localMotorChannelA, int localMotorChannelB, int localState1, int localState2) 
{
  analogWrite(localMotorChannelA, localState1);
  analogWrite(localMotorChannelB, localState2);
}



//Function to read left encoder
void LeftEncoder() 
{
  int channelBRead = digitalRead(leftEncoderChannelB);

  if (channelBRead > 0)
    leftPosition++;
  else
    leftPosition--;
}

//Function to read right encoder
void RightEncoder() 
{
  int channelBRead = digitalRead(rightEncoderChannelB);

  if (channelBRead > 0)
    rightPosition++;
  else
    rightPosition--;
}


//Function to read ultrasonic distance
long Distance(const int localTrig, const int localEcho) 
{
  float localDistanceCM;
  long duration, localDistanceMM;
  digitalWrite(localTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(localTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(localTrig, LOW);

  duration = pulseIn(localEcho, HIGH, 30000);
  localDistanceCM = duration / 5.83;
  localDistanceMM = round(localDistanceCM / 5.0) * 5;
  return localDistanceMM;
}


float PidCalculations(long &localError, long localPositionTarget, long localPosition, float localProportional, float localKp, unsigned long &localPrevTime, float localDerivative, float localKd, long &localPrevError)
{
  localError = localPositionTarget - localPosition;                                               
  localProportional = localKp * localError;

  if(millis() - localPrevTime == 0)
    localKd = 0;
  else
    localDerivative = (localKd * (localError - localPrevError) / (millis() - localPrevTime));

  return localProportional + localDerivative;
}



//Function to move the motors a certain distance (based on encoders)
void Movement(long leftPositionTarget, long rightPositionTarget)
{
  while (leftPosition != leftPositionTarget || rightPosition != rightPositionTarget)
  {
    leftUp = constrain(abs(PidCalculations(leftError, leftPositionTarget, leftPosition, leftProportional, leftKp, leftPrevTime, leftDerivative, leftKd, leftPrevError)), 0, motorPower);
    rightUp = constrain(abs(PidCalculations(rightError, rightPositionTarget, rightPosition, rightProportional, rightKp, rightPrevTime, rightDerivative, rightKd, rightPrevError)), 0, motorPower);
       
    if (leftPosition < leftPositionTarget)                                                                                    //Left Motor Conditions
      Control(leftMotorChannelA, leftMotorChannelB, leftUp, 0);
    if (leftPosition > leftPositionTarget)
      Control(leftMotorChannelA, leftMotorChannelB, 0, leftUp);

    if (rightPosition < rightPositionTarget)                                                                                   //Right Motor Conditions
      Control(rightMotorChannelA, rightMotorChannelB, rightUp, 0);
    if (rightPosition > rightPositionTarget)
      Control(rightMotorChannelA, rightMotorChannelB, 0, rightUp);
    
    leftPrevTime = millis();                      //Setting previous values for next calculation cycle
    leftPrevError = leftError;
    rightPrevTime = millis();
    rightPrevError = rightError;

    //Print();                                      //Print to Serial Monitor

    if (millis() - timer > timeout && abs(leftPosition) >= abs(leftPositionTarget) * minError && abs(leftPosition) <= abs(leftPositionTarget) * maxError && abs(rightPosition) >= abs(rightPositionTarget) * minError && abs(rightPosition) <= abs(rightPositionTarget) * maxError)
    {
      Serial.print("Left Position: ");
      Serial.print(leftPosition);
      Serial.print(" | ");
      Serial.print("Right Position: ");
      Serial.println(rightPosition);
      Serial.println("Timout");
      break;
    }
  }

  Control(leftMotorChannelA, leftMotorChannelB, 0, 0);              //If leftPosition and rightPosition reach target
  Control(rightMotorChannelA, rightMotorChannelB, 0, 0);
  //Print(); 
  //Serial.println("Movement Done");
  leftPosition = 0;
  rightPosition = 0;
  delay(1000);
}


//Function to read ultrasonics only after sufficient wait time
void LeePing(byte localTrig, byte localEcho, long &localDistance)
{
  if (millis() - lastPingTime >= pingInterval) 
    {
      lastPingTime = millis();
      distanceCheck = Distance(localTrig, localEcho);
    }

  if (distanceCheck > 0)
    localDistance = distanceCheck;
}


bool CellCheck(int openCell)
{
  int targetRow = mazeMapRow;
  int targetColumn = mazeMapColumn;

  cellCheck = direction + openCell;
  if (cellCheck < 0)
    cellCheck += 4;
  else if (cellCheck > 3)
    cellCheck -= 4;

  switch(cellCheck)
  {
    case 0:
      targetRow--;
      break;
    case 1:
      targetColumn++;
      break;
    case 2:
      targetRow++;
      break;
    case 3:
      targetColumn--;
      break;
  }
  
  if (targetRow >= 0 && targetRow <= 5 && targetColumn >= 0 && targetColumn <= 3)
    cellState = mazeMap[targetRow][targetColumn];
  else
    cellState = 1;

  return cellState;
}


void openPath(short directionChange)
{
  routePos++;
  route[routePos] = directionChange;

  switch(directionChange)
  {
    case 0:
      Serial.println("Starting frontOpen...\n\n\n");
      break;
    case -1:
      Serial.println("Starting leftOpen...\n\n\n");
      break;
    case 1:
      Serial.println("Starting rightOpen...\n\n\n");
      break;
  }

  if (directionChange != 0)
  {
    timer = millis();                           //Turn
    Movement(directionChange * turnConstant, -1 * directionChange * turnConstant);   

    switch(directionChange)                     //Change direction reading CCW
    {
      case -1:
        Serial.println("Turned left...\n\n");
        direction--;
        break;
      case 1:
        Serial.println("Turned right...\n\n");
        direction++;
        break;
    }
    delay(500);
  }
          
  if (direction < 0)
    direction += 4;
  else if (direction > 3)
    direction -= 4;
  Serial.print("Direction: ");
  Serial.println(direction);
  Serial.print("Direction Set\n\n");
  delay(500);

  timer = millis();                         //Go straight
  Movement(straightConstant, straightConstant);                         
  Serial.println("Went straight\n\n");
  delay(500);

  switch(direction)
  {
    case 0:
      mazeMapRow--;
      break;
    case 1:
      mazeMapColumn++;
      break;
    case 2:
      mazeMapRow++;
      break;
    case 3:
      mazeMapColumn--;
      break;
  }

  mazeMap[mazeMapRow][mazeMapColumn] = 1;
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      Serial.print(mazeMap[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("\n\n\n\n");
  delay(500);

  for (int i = 23; i >= 0; i--)
  {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(route[i]);
  }
  delay(500);  
  cellCheckTimer = millis();
}


void BackTrack()
{
  Serial.println("\n\n\nBacktracking...\n\n");

  while (routePos >= 0)
  {
    timer = millis();                                 //Go backwards
    Movement(-1 * straightConstant, -1 * straightConstant);                         
    Serial.println("Went backwards\n\n");
    delay(500);
    
    switch(route[routePos])
    {
      case 0:
        break;
      case 1:
        Movement(-1 * turnConstant, turnConstant);
        Serial.println("Turned left");
        break;
      case -1:
        Movement(turnConstant, -1 * turnConstant);
        Serial.println("Turned right");
        break;
    }
    delay(500);

    routePos--;

    mazeMap[mazeMapRow][mazeMapColumn] = 1;
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        Serial.print(mazeMap[i][j]);
        Serial.print(" ");
      }
      Serial.println();
    }

    Serial.println("\n\n\n\n");
    delay(2000);

    for (int i = 23; i >= 0; i--)
    {
      Serial.print(i);
      Serial.print(": ");
      Serial.println(route[i]);
    }
  }

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      if (i == 5 && j == 2)
        mazeMap[i][j] = 1;
      else
        mazeMap[i][j] = 0;
    }
  }

  for (int i = 23; i >= 0; i--)
  {
    route[i] = 0;
  }

  routePos = -1;
  mazeMapRow = 5;
  mazeMapColumn = 2;
  direction = 0;
  cellCheckTimer = millis();
}

void BackTrack1()
{
  Serial.println("\n\n\nBacktracking...\n\n");

  timer = millis();                                 //Go backwards
  Movement(-1 * straightConstant, -1 * straightConstant);                         
  Serial.println("Went backwards\n\n");
  delay(500);

  switch(direction)
  {
    case 0:
      mazeMapRow++;
      break;
    case 1:
      mazeMapColumn--;
      break;
    case 2:
      mazeMapRow--;
      break;
    case 3:
      mazeMapColumn++;
      break;
  }
    
  switch(route[routePos])
  {
    case 0:
      break;
    case 1:
      Movement(-1 * turnConstant, turnConstant);
      direction--;
      Serial.println("Turned left");
      break;
    case -1:
      Movement(turnConstant, -1 * turnConstant);
      direction++;
      Serial.println("Turned right");
      break;
  }
  delay(500);

  route[routePos] = 0;
  routePos--;

  if (direction < 0)
    direction += 4;
  else if (direction > 3)
    direction -= 4;

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      Serial.print(mazeMap[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("\n\n\n\n");
  delay(500);

  for (int i = 23; i >= 0; i--)
  {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(route[i]);
  }

  Serial.print(mazeMapRow);
  Serial.print(", "); 
  Serial.println(mazeMapColumn);
  Serial.println("\n\n");
  delay(2000);
  cellCheckTimer = millis();
}


void Print()
{
  /*
  Serial.print("Row: ");
  Serial.print(i);
  Serial.print(" | ");
  Serial.print("Column: ");
  Serial.print(j);
  Serial.print(" | ");
  Serial.println(mazeMap[i][j]);
  delay(1000);
  
  Serial.print(-1100);
  Serial.print(",");
  Serial.print(1100);
  Serial.print(",");
  Serial.print(leftPosition);
  Serial.print(",");
  Serial.println(rightPosition);
  
  Serial.print(255);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(leftPosition);
  */
  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print(" | ");
  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print(" | ");
  Serial.print("Right: ");
  Serial.println(rightDistance);
  
}



void setup() {
  Serial.begin(9600);
  //Motor Pin Setup
  pinMode(leftMotorChannelA, OUTPUT);
  pinMode(leftMotorChannelB, OUTPUT);
  pinMode(leftMotorChannelA, OUTPUT);
  pinMode(leftMotorChannelB, OUTPUT);

  //Initialize Motor Pins
  digitalWrite(leftMotorChannelA, LOW);
  digitalWrite(leftMotorChannelB, LOW);
  digitalWrite(rightMotorChannelA, LOW);
  digitalWrite(rightMotorChannelB, LOW);

  //Sensor Pin Setup
  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  pinMode(rightTrig, OUTPUT);
  pinMode(rightEcho, INPUT);
  pinMode(frontBottomTrig, OUTPUT);
  pinMode(frontBottomEcho, INPUT);

  digitalWrite(frontTrig, LOW);
  digitalWrite(leftTrig, LOW);
  digitalWrite(rightTrig, LOW);
  digitalWrite(frontBottomTrig, LOW);

  //Interrupts
  attachInterrupt(digitalPinToInterrupt(leftEncoderChannelA), LeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderChannelA), RightEncoder, RISING);  
  delay(1000);
}



void loop() {
  /*
  Movement(straightConstant, straightConstant);
  Print();
  delay(1000);
  */
  frontDistance = Distance(frontTrig, frontEcho);
  leftDistance = Distance(leftTrig, leftEcho);
  rightDistance = Distance(rightTrig, rightEcho);
  frontBottomDistance = Distance(frontBottomTrig, frontBottomEcho);

  if (frontDistance - frontBottomDistance > 100)
  {
    BackTrack1();
  }

  if (leftDistance > 100 && CellCheck(-1) == 0)
    openPath(-1);
  else if (frontDistance > 100 && CellCheck(0) == 0)
    openPath(0);
  else if (rightDistance > 100 && CellCheck(1) == 0)
    openPath(1);

  if (millis() - cellCheckTimer > 1000 && routePos >= 0)
  {
    BackTrack1();
  }

  Print();
  
  /*
  timer = millis();
  Movement(1000);
  */
}