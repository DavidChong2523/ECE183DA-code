// Main code for AsparaBot Prototype
// MAE 162D Spring 2022
////////////////////////////////////
#include <Servo.h>

// Define motor connections:
#define dirPinZ 2
#define stepPinZ 3
#define dirPinY 4
#define stepPinY 5
#define limSwitchY 6
#define limSwitchZ 7
#define cutterPin 9
#define gripperPin 10

// Motor-specific constants
const int microstepZ = 1; // Microstep setting (1 = none, 2 = half-step, etc.)
const unsigned int maxMoveDurationZ = 31000*microstepZ; // Maximum steps Z motor is allowed to move at once
const unsigned int stepsOppositeZ = 30000*microstepZ; // Steps from home to opposite end of Z-axis
const unsigned int cuttingPosZ = 17000*microstepZ; // Steps to "ground"
const int stepDelayZ = 650;

const int microstepY = 2;
const unsigned int maxMoveDurationY = 1800*microstepY; // Maximum steps Y motor is allowed to move at once
const unsigned int stepsOppositeY = 1725*microstepY; // Steps from home to opposite end of Y-axis
const int stepDelayY = 1500;
int yPos = 0;

int inputPos; // Serial user input for position

Servo gripper;  // create servo object to control a servo
Servo cutter;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  
  // Declare output pins
  pinMode(stepPinZ, OUTPUT);
  pinMode(dirPinZ, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinY, OUTPUT);

  // Declare input pins
  pinMode(limSwitchY, INPUT);
  pinMode(limSwitchZ, INPUT);

  // Set the spinning direction towards the motor (home)
  digitalWrite(dirPinZ, LOW);
  digitalWrite(dirPinY, LOW);

  // Set motors to home
  homeZMotor();
  delay(1000);
  homeYMotor();
  delay(1000);

  // Initialize actuator servos
  cutter.write(0);
  gripper.write(0);
  delay(1000);
  cutter.attach(cutterPin);
  gripper.attach(gripperPin);
  delay(1000);
}

void loop() {
  while(!Serial.available()); 
  inputPos = Serial.readString().toInt();
  Serial.println("Cutting asparagus at: " + String(inputPos) + "...");
  // moveYTo(inputPos);
  cutAsparagusAt(inputPos);
  Serial.println("Finished.");
}



void cutAsparagusAt(unsigned int targetPos) {
  // move Y to targetPos
  moveYTo(targetPos);
  delay(5000);
  // move Z to cuttingPosZ
  moveZTo(cuttingPosZ);
  delay(1000);
  // close gripper
  grip_close(0,50);
  delay(1000);
  // activate cutter
  cut_cycle(0,110);  // close and open the blade
  delay(1000);
  /*
  cut_cycle(0,110);  // close and open the blade
  delay(100);
  cut_cycle(0,110);  // close and open the blade
  delay(1000);
  */
  // move Z to home
  homeZMotor();
  delay(1000);
  // move Y to stepsOppositeY
  moveYTo(stepsOppositeY);
  delay(1000);
  // release gripper
  grip_open(0,50);
  delay(1000);
}

// GANTRY HELPER FUNCTIONS
////////////////////////////

void homeYMotor() {
  int i = 0;
  digitalWrite(dirPinY, LOW);
  while(i < maxMoveDurationY) {
    if (digitalRead(limSwitchY) == HIGH) {
      // Serial.println("Pressed.");
      break;
    }
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelayY);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelayY);
    i++;
  }
}

void moveYTo(unsigned int targetPos) {
  // Check for valid input
  if (targetPos > stepsOppositeY) {
    targetPos = stepsOppositeY;
  }

  // Determine direction of travel
  if (yPos < targetPos) {
    digitalWrite(dirPinY, HIGH);
  }
  else if (yPos > targetPos) { 
    digitalWrite(dirPinY, LOW);
  }

  // Determine distance of travel
  int d = yPos - targetPos;
  d = abs(d);
  for (int i = 0; i < d; i++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelayY);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelayY);
  }
  yPos = targetPos;
}

void homeZMotor() {
  int i = 0;
  digitalWrite(dirPinZ, LOW);
  while(i < maxMoveDurationZ) {
    if (digitalRead(limSwitchZ) == HIGH) {
      // Serial.println("Pressed.");
      break;
    }
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayZ);
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelayZ);
    i++;
  }
}

void moveZTo(unsigned int pos) {
  // Check for valid input
  if (pos > maxMoveDurationZ) {
    pos = maxMoveDurationZ;
  }
  
  digitalWrite(dirPinZ, HIGH);
  for (int i = 0; i < pos; i++) {
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayZ);
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelayZ);
  }
}

// ACTUATOR HELPER FUNCTIONS
//////////////////////////////

void grip_close(int min, int max) {
  for (int pos = min; pos <= max; pos += 1) { // goes from min degrees to max degrees
    // in steps of 1 degree
    gripper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}


void grip_open(int min, int max)  {
  for (int pos = max; pos >= min; pos -= 1) { // goes from max degrees to min degrees
    gripper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void cut_cycle(int min, int max){
//  for (int pos = min; pos <= max; pos += 5) { // goes from min degrees to max degrees
//    // in steps of 5 degrees
//    cutter.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
  cutter.write(max);
  delay(2000);
  
  for (int pos = max; pos >= min; pos -= 1) { // goes from max degrees to min degrees
    cutter.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
