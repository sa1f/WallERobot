#define NOFIELD 90L
#define TOMILLIGAUSS 1953L
#include <LiquidCrystal.h>
#include <Servo.h>

//---------PIN DEFINITION----------------
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); //RS, EN, D4, D5, D6, D7


const int LEFT_OPTIC_PIN = A3;
const int RIGHT_OPTIC_PIN = A2;
const int TEMP_SENSOR_PIN = A1;
Servo myservo; //SERVO WILL BE CONNECTED ON A0

//const int BLUETOOTH_PIN = 0;
const int PUSH_BUTTON_PIN = 1;
const int ULTRASONIC_ECHO_PIN = 2;
const int ULTRASONIC_TRIG_PIN = 3;
const int MOTOR_E1_PIN = 5;
const int MOTOR_M1_PIN = 4;
const int MOTOR_E2_PIN = 6;
const int MOTOR_M2_PIN = 7;


//Motor Driver Constants
const double LEFT_MOTOR_ADJUSTMENT = 0.97;
const double RIGHT_MOTOR_ADJUSTMENT = 1.0;
const double MAX_SPEED = 255;
const int TURN_TIME = 800; //how long it takes the robot to spin 90 deg

//DIRECTION CONSTANTS  
const boolean RIGHT = true;
const boolean LEFT = false;
const boolean FORWARD = true;
const boolean BACK = false;

//LCD Constants
const int FIRST_COL = 0;
const int TOP_ROW = 1;
const int BOTTOM_ROW = 1;

//State changing
int currentState = 0;   // counter for the number of button presses
int inputState = 0;         // current state of the button
int lastInputState = -1;     // previous state of the button

//Autonomous Mode
const double D_MAX = 40; //Distance when speed starts being adjusted
const double D_MIN = 10; //Distance robot should stop

//Variables for checking whether robot is stuck
const int D_STALL = 3;  
const int STUCK_THRESHOLD = 60;
int isStopped = 0;

double temperature;

//Line Follow Mode
const int LIGHT_THRESHOLD = 200;
const int LINE_SPEED = 150;

int leftWhite = 0;
int rightWhite = 0;
int leftSensor;
int rightSensor;

//--------------Robot States
int robotState;
int lastRobotState = 0;
const int FULL_SPEED = 0;
const int TURN_LEFT = 1;
const int TURN_RIGHT = 2;

//---------------Speed modelling variables
double A;
double B;

//--------------Bluetooth variables
byte byteRead;

void setup() {

  //Ultrasonic setup
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);

  //Servo setup
  myservo.attach(A0);

  //Light Sensor Initialization
  leftWhite = analogRead(LEFT_OPTIC_PIN);
  rightWhite = analogRead(RIGHT_OPTIC_PIN);

  //Motor Setup
  pinMode(MOTOR_M1_PIN, OUTPUT);
  pinMode(MOTOR_M2_PIN, OUTPUT);

  //LCD Setup
  lcd.begin(16, 2);

  //Button setup
  pinMode(PUSH_BUTTON_PIN, INPUT);

  //Setting up the modelling variables
  A = MAX_SPEED / (pow(D_MAX, 2.0) - pow(D_MIN, 2.0));
  B = (MAX_SPEED * pow(D_MIN, 2.0) / (pow(D_MIN, 2.0) - pow(D_MAX, 2.0)));

  //Temperature initialization
  temperature = (( analogRead(TEMP_SENSOR_PIN) / 1024.0) * 5000) / 10;
  
  // Initialize serial for bt communication
  Serial.begin(9600);
  
}

void loop() {
  
  //Partial Extra Functionality (NOT COMPLETED)
  //<!---------------
  if (Serial.available()) {
    /* read the most recent byte */
    byteRead = Serial.read();
    /*ECHO the value that was read, back to the serial port. */
    switch (byteRead) {
      case 'a': currentState = 0; break;
      case 'l': currentState = 1; break;
      case 'c': currentState = 2; break;
    } 
  }
  //------------------!>

  //Checks for button input for mode change
  inputState = digitalRead(PUSH_BUTTON_PIN);
  
  //Checks if there is a state change
  if (inputState != lastInputState) {
    currentState = (currentState + 1) % 3;
    lcd.clear();
    switch (currentState) {
      case 0: lcd.print("Autonomous Mode"); break;
      case 1: lcd.print("Line Follow Mode"); break;
      case 2: lcd.print("Angular Autonomous");
              lcd.setCursor(FIRST_COL, BOTTOM_ROW);
              lcd.print("Mode");
              break;
      default: lcd.print("UNKNOWN MODE");

    }
    delay(1000);
  }

  lastInputState = inputState;

  switch (currentState) {
    case 0: autonomousLoop(); break;
    case 1: lineFollowLoop(); break;
    case 2: angularAutonomousLoop(); break;
  }

}

/**
 * Loop for binary autonomous functionality
 * Turns LEFT or RIGHT.
 */
void autonomousLoop() {
  myservo.write(90);
  
  //get rid of interference
  long distance = max(max(get_distance(), get_distance()), get_distance());

  //Retry when the distance isn't within the data-sheet expected values
  while (distance < 2 || distance > 400) {
    distance = get_distance();
  }

  //Determines whether the robot is stalled
  if ((distance < D_MAX ) && (D_STALL > abs(distance - get_distance()))) {
    isStopped++;
  }

  //Adjusts the speed of robot based on distance of object in front
  int robotSpeed = adjustSpeed(distance);
  moveInDirection(FORWARD, robotSpeed);

  //Outputs status on LCD
  lcd.clear();
  lcd.print("Distance: " + String(distance));
  lcd.setCursor(FIRST_COL, BOTTOM_ROW);
  lcd.print("Speed Ratio: " + String(robotSpeed));

  //Turning Logic
  if (distance < D_MIN || isStopped > STUCK_THRESHOLD) {
    lcd.clear();
    if (isStopped > STUCK_THRESHOLD) {
      lcd.print("Stuck at: " + String(distance));
    } else {
      lcd.clear();
      lcd.print("Object detected at:");
      lcd.setCursor(FIRST_COL, BOTTOM_ROW);
      lcd.print("Dist: " + String(distance));
    }
    halt();

    //Determines the direction the robot should turn
    int escapeAngle = findEscapeRoute();
    lcd.setCursor(FIRST_COL, BOTTOM_ROW);
    lcd.print("Turn: " + String(escapeAngle));
    rotateAngle(escapeAngle);
    isStopped = 0;
  }
}


/**
 * Loop for angular autonomous pathfinding
 * Turns at a specific angle between 0 to 180
 */
void angularAutonomousLoop() {
  myservo.write(90);
  
  //get rid of interference
  long distance = max(max(get_distance(), get_distance()), get_distance());

  while (distance < 2 || distance > 400) {
    distance = get_distance();
  }

  //Determines whether the robot is stalled
  if ((distance < D_MAX ) && (D_STALL > abs(distance - get_distance()))) {
    isStopped++;
  }

  //Adjusts the speed of robot based on distance of object in front
  int robotSpeed = adjustSpeed(distance);
  moveInDirection(FORWARD, robotSpeed);

  //Outputs status on LCD
  lcd.clear();
  lcd.print("Distance: " + String(distance));
  lcd.setCursor(FIRST_COL, BOTTOM_ROW);
  lcd.print("Speed Ratio: " + String(robotSpeed));

  //Turning Logic
  if (distance < D_MIN || isStopped > STUCK_THRESHOLD) {
    lcd.clear();
    if (isStopped > STUCK_THRESHOLD) {
      lcd.print("Stuck at: " + String(distance));
    } else {
      lcd.clear();
      lcd.print("Object detected at:");
      lcd.setCursor(FIRST_COL, BOTTOM_ROW);
      lcd.print("Dist: " + String(distance));
    }
    halt();

    //Determines the direction the robot should turn
    int escapeAngle = findEscapeAngle();
    lcd.setCursor(FIRST_COL, BOTTOM_ROW);
    lcd.print("EA: " + String(escapeAngle));
    rotateAngle(escapeAngle);
    isStopped = 0;
  }
}

/**
 * Loop for line following functionality
 */
void lineFollowLoop() {
  //Multiple reads to reduce interference
  analogRead(LEFT_OPTIC_PIN);
  leftSensor = analogRead(LEFT_OPTIC_PIN);
  analogRead(RIGHT_OPTIC_PIN);
  rightSensor = analogRead(RIGHT_OPTIC_PIN);

  //if left sensor detects path, turn left
  if (leftSensor > leftWhite + LIGHT_THRESHOLD) {
    moveLeftWheel(FORWARD, 0);
    moveRightWheel(FORWARD, MAX_SPEED);
    robotState = TURN_LEFT;
  } else if (rightSensor > rightWhite + LIGHT_THRESHOLD) {
    // if right sensor detects path, turn right
    moveRightWheel(FORWARD, 0);
    moveLeftWheel(FORWARD, MAX_SPEED);
    robotState = TURN_RIGHT;
  } else {
    // if nothing is detected, move straight
    moveInDirection(FORWARD, LINE_SPEED);
    robotState = FULL_SPEED;
  }

  //Prints out current movement to LCD
  if (robotState != lastRobotState) {
    lcd.clear();
    switch (robotState) {
      case FULL_SPEED: lcd.print("FULL SPEED"); break;
      case TURN_LEFT: lcd.print("TURN LEFT"); break;
      case TURN_RIGHT: lcd.print("TURN RIGHT"); break;
      default: lcd.print("Invalid");
    }
    lastRobotState = robotState;
  }
  
}

/*
 * Debug code for motors 
 */
void motorTestLoop() {
  lcd.clear();
  lcd.print("Halting");
  delay(1000);
  halt();
  lcd.clear();
  lcd.print("Begin Test");
  delay(1000);
  lcd.clear();
  lcd.print("90deg right");
  rotateAngle(0);
  delay(1000);
  lcd.clear();
  lcd.print("90deg left");
  rotateAngle(180);
  delay(1000);
  lcd.clear();
  lcd.print("180deg left");
  turnLeft();
  turnLeft();
  delay(1000);
  lcd.clear();
  lcd.print("180deg right");
  turnRight();
  turnRight();
  delay(1000);
}

/**
   Moves the robot in a specified direction at a specified speed
   @param dir - the direction the robot is heading
                  true for forward, false for backwards
   @param robotSpeed - the speed the robot is moving
*/

void moveInDirection(boolean dir, int robotSpeed) {
  moveLeftWheel(dir, robotSpeed);
  moveRightWheel(dir, robotSpeed);
}


/**
   Stops motors of the robot
*/
void halt() {
  analogWrite(MOTOR_E1_PIN, 0);
  analogWrite(MOTOR_E2_PIN, 0);
}

/**
   Rotates the robot in place
   @param dir - the direction the robot is turning
                  true for right, false for left
   @param duration - the duration the robot is turning
*/
void rotate(boolean dir, int duration) {
  moveRightWheel(!dir, MAX_SPEED * 0.5);
  moveLeftWheel(dir, MAX_SPEED * 0.5);
  delay(duration);
  halt();
}

/**
   Turns the robot left
*/
void turnLeft() {
  rotateAngle(180);
}

/**
   Turns the robot right
*/
void turnRight() {
  rotateAngle(0);
}

/**
   Moves the right wheel in a direction with a speed
   @param dir - the direction the wheel is moving
                  true for forward, false for backward
   @param robotSpeed - the speed the wheel is moving
*/
void moveRightWheel(boolean dir, int robotSpeed) {
  digitalWrite(MOTOR_M1_PIN, dir);
  analogWrite(MOTOR_E1_PIN, robotSpeed * RIGHT_MOTOR_ADJUSTMENT);
}

/**
   Moves the left wheel in a direction with a speed
   @param dir - the direction the wheel is moving
                  true for forward, false for backward
   @param robotSpeed - the speed the wheel is moving
*/
void moveLeftWheel(boolean dir, int robotSpeed) {
  digitalWrite(MOTOR_M2_PIN, dir);
  analogWrite(MOTOR_E2_PIN, robotSpeed * LEFT_MOTOR_ADJUSTMENT);
}

/**
   Rotates the robot a specific angle in a direction.
   @Param angle - the angle it turns to
      - 180 for left, 0 for right
*/
void rotateAngle(int angle) {
  if (angle < 0 || angle > 180) return;
  if (angle > 90) {
    rotate(LEFT, (1.0 / 90 * angle - 1) * TURN_TIME);
  } else {
    rotate(RIGHT, (-1.0 / 90 * angle + 1) * TURN_TIME);
  }
}

/**
   Adjust the speed of the robot depending on the distance obtained
   @param distance - the distance of the object in front of robot
   @return speed - the speed of the robot
*/
int adjustSpeed(int distance) {
  int robotSpeed;
  if (distance > D_MAX) {
    robotSpeed = MAX_SPEED;
  }
  else if (distance < D_MIN) {
    robotSpeed = 0;
  } else {
    robotSpeed = A * pow(distance, 2.0) + B;
  }
  return robotSpeed;
}


/**
   Returns the distance in centimeters of the object
   in front of the ultrasonic sensor
*/
unsigned long get_distance() {

  float speed_of_sound = 331.5 + (0.6 * temperature);
  float denom = (20000.0 / speed_of_sound);
  //Pulse the TRIG on the ULTRASONIC sensor
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  //Read the ECHO signal from the ULTRASONIC sensor
  unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  unsigned long distance = duration / denom;
  return distance;
}

/** 
    Returns the best direction the robot to turn
    from either left or right 
  */
int findEscapeRoute() {
  int distance;
  int maxDistance = 0;
  int maxAngle = 0;
  myservo.write(0);
  delay(1000);
  distance = get_distance();
  myservo.write(180);
  delay(1000);
  if (get_distance() > distance){
    maxAngle = 180;
  }
  myservo.write(90);
  return maxAngle;
}

/* 
  Returns the best angle the robot to turn
  from a range of 0 to 180.
*/
int findEscapeAngle() {
  int distance;
  int maxDistance = 0;
  int maxAngle = 0;
  
  for (int p = 0; p <= 180; p++) {
    myservo.write(p);
    delay(10);
    if(p == 0){
      delay(20);
    }
    distance = get_distance();
    if(distance > maxDistance){
      maxDistance = distance;
      maxAngle = p;
    }
  
  }
  return maxAngle;
}
