#define NOFIELD 90L
#define TOMILLIGAUSS 1953L
#include <LiquidCrystal.h>
#include <Servo.h>

//PIN DEFINITION
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); //RS, EN, D4, D5, D6, D7
const int FIRST_COL = 0;
const int TOP_ROW = 1;
const int BOTTOM_ROW = 1;

const int MOTOR_E1_PIN = 5;
const int MOTOR_M1_PIN = 4;
const int MOTOR_E2_PIN = 6;
const int MOTOR_M2_PIN = 7;
const double LEFT_MOTOR_ADJUSTMENT = 0.97;
const double RIGHT_MOTOR_ADJUSTMENT = 1.0;

const int ULTRASONIC_TRIG_PIN = 3;
const int ULTRASONIC_ECHO_PIN = 2;
const int PUSH_BUTTON_PIN = 1;
//const int BLUETOOTH_PIN = 0;


const int HALL_EFFECT_LEFT_PIN = A5;
const int HALL_EFFECT_RIGHT_PIN = A4;
const int LEFT_OPTIC_PIN = A3;
const int RIGHT_OPTIC_PIN = A2;
const int TEMP_SENSOR_PIN = A1;
Servo myservo; //SERVO WILL BE CONNECTED ON A0

//CONSTANTS
const boolean RIGHT = true;
const boolean LEFT = false;
const boolean FORWARD = true;
const boolean BACK = false;

const double MAX_SPEED = 255;
const int TURN_TIME = 800; //how long it takes the robot to spin 90 deg

//Distance constants
const double D_MAX = 30;
const double D_MIN = 10;
const int D_STALL = 3;
const int STUCK_THRESHOLD = 150;

const int LIGHT_THRESHOLD = 200;
const int LINE_SPEED = 140;
int leftWhite = 0;
int rightWhite = 0;
int leftSensor;
int rightSensor;

//Robot States
int robotState;
int lastRobotState = 0;
const int FULL_SPEED = 0;
const int TURN_LEFT = 1;
const int TURN_RIGHT = 2;

// Variables will change:
int currentState = 0;   // counter for the number of button presses
int inputState = 0;         // current state of the button
int lastInputState = -1;     // previous state of the button
int isStopped = 0;

//Rotary Encoder Variables
long long rightWheelTime;
long long leftWheelTime;
int rightWheelSpeed;
int leftWheelSpeed;

//Speed modelling variables
double A;
double B;
const float wheelCircumference = 2 * PI * 0.031;

long long _time = 0;
int _speed = 150;
long long _delay = 5000;
float temperature;

void setup() {
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);

  myservo.attach(A0);

  leftWhite = analogRead(LEFT_OPTIC_PIN);
  rightWhite = analogRead(RIGHT_OPTIC_PIN);

  pinMode(MOTOR_M1_PIN, OUTPUT);
  pinMode(MOTOR_M2_PIN, OUTPUT);

  lcd.begin(16, 2);
  pinMode(PUSH_BUTTON_PIN, INPUT);

  //Setting up the modelling variables
  A = MAX_SPEED / (pow(D_MAX, 2.0) - pow(D_MIN, 2.0));
  B = (MAX_SPEED * pow(D_MIN, 2.0) / (pow(D_MIN, 2.0) - pow(D_MAX, 2.0)));

  _time = millis();
  rightWheelTime = millis();
  leftWheelTime = millis();

  temperature = (( analogRead(TEMP_SENSOR_PIN) / 1024.0) * 5000) / 10;
}

void loop() {
  inputState = digitalRead(PUSH_BUTTON_PIN);
  
  if (inputState != lastInputState) {
    currentState = (currentState + 1) % 3;
    lcd.clear();
    switch (currentState) {
      case 0: lcd.print("Autonomous Mode"); break;
      case 1: lcd.print("Line Follow Mode"); break;
      case 2: lcd.print("BT Controller Mode"); break;
      default: lcd.print("UNKNOWN MODE");

    }
    delay(1000); //Debouncing
  }

  lastInputState = inputState;

  switch (currentState) {
    case 0: autonomous_loop(); break;
    case 1: line_follow_loop(); break;
    case 2: bluetooth_loop(); break;
  }
  
  /*
    long left = get_gauss(true);
    long right = get_gauss(false);

    lcd.clear();
    //lcd.setCursor(FIRST_COL, TOP_ROW);
    lcd.print("Lef:"+String(left));
    lcd.print("Rig:"+String(right));

    moveInDirection(FORWARD, 250);
    if(left < 0){
    lcd.clear();
    lcd.setCursor(FIRST_COL, BOTTOM_ROW);
    lcd.print("L ");
    delay(10);
    }

    if(right < 0){
    lcd.clear();
    lcd.setCursor(2, BOTTOM_ROW);
    lcd.print("R");
    delay(10);
    }
  */
}

void autonomous_loop() {
  myservo.write(90);
  //get rid of interference
  long distance = max(max(get_distance(), get_distance()), get_distance());
  //long distance = get_distance();
  while (distance < 2 || distance > 400) {
    distance = get_distance();
  }

  if ((distance < D_MAX ) && (D_STALL > abs(distance - get_distance()))) {
    isStopped++;
  }
  int robotSpeed = adjustSpeed(distance);
  moveInDirection(FORWARD, robotSpeed);
  lcd.clear();
  lcd.print("Distance: " + String(distance));
  lcd.setCursor(FIRST_COL, BOTTOM_ROW);
  lcd.print("Speed: " + String(robotSpeed));
  lcd.print("  S: " + String(isStopped));
  if (distance < D_MIN || isStopped > STUCK_THRESHOLD) {
    lcd.clear();
    if (isStopped > 50) {
      lcd.print("Stuck at: " + String(distance));
    } else {
      lcd.clear();
      lcd.print("Object detected");
      lcd.setCursor(FIRST_COL, BOTTOM_ROW);
      lcd.print("Dist: " + String(distance));
      lcd.print(" S:" + String(isStopped));
    }
    halt();
    int escapeAngle = findEscapeRoute();
    lcd.setCursor(FIRST_COL, BOTTOM_ROW);
    lcd.print("EA: " + String(escapeAngle));
    rotateAngle(escapeAngle);
    isStopped = 0;
  }
}

void line_follow_loop() {
  if (millis() > (_time + _delay)) {
    _speed = _speed + 10;
    if (_speed > 255) {
      _speed = 255;
    }
    _time = millis();
  }
  analogRead(LEFT_OPTIC_PIN);
  leftSensor = analogRead(LEFT_OPTIC_PIN);
  analogRead(RIGHT_OPTIC_PIN);
  rightSensor = analogRead(RIGHT_OPTIC_PIN);
  if (leftSensor > leftWhite + LIGHT_THRESHOLD) {
    moveLeftWheel(FORWARD, 0);
    moveRightWheel(FORWARD, _speed);
    robotState = TURN_LEFT;
  }
  // if right sensor detects path, move right
  else if (rightSensor > rightWhite + LIGHT_THRESHOLD) {
    moveRightWheel(FORWARD, 0);
    moveLeftWheel(FORWARD, _speed);
    robotState = TURN_RIGHT;
  }
  // if nothing is detected, move straight
  else {
    moveInDirection(FORWARD, _speed);
    robotState = FULL_SPEED;
  }

  if (robotState != lastRobotState) {
    lcd.clear();
    switch (robotState) {
      case FULL_SPEED: lcd.print("FULL SPEED"); break;
      case TURN_LEFT: lcd.print("TURN LEFT"); break;
      case TURN_RIGHT: lcd.print("TURN RIGHT"); break;
      default: lcd.print("Invalid");
    }
    lcd.setCursor(FIRST_COL, BOTTOM_ROW);
    lcd.print(String(_speed));
    
    lastRobotState = robotState;
  }
}

void bluetooth_loop() {
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
   Constantly updates the velocity of each wheel of the robot
*/
/*
void determine_velocity() {
  if (analogRead(HALL_EFFECT_RIGHT_PIN) < 0) {
    rightWheelSpeed = wheelCircumference / (millis() - rightWheelTime);
    rightWheelTime = millis();
  }

  if (analogRead(HALL_EFFECT_LEFT_PIN) < 0) {
    leftWheelSpeed = wheelCircumference / (millis() - leftWheelTime);
    leftWheelTime = millis();
  }
}
*/
/**
   Moves the robot in a specified direction at a specified speed
   @param dir - the direction the robot is heading
                  true for forward, false for backwards
   @param robotSpeed - the speed the robot is moving
*/

void moveInDirection(boolean dir, int robotSpeed) {
  /*if (leftWheelSpeed == rightWheelSpeed) {
    //No calibration needed
  } else if (leftWheelSpeed < rightWheelSpeed) {
    rightWheelSpeed--;
  } else {
    leftWheelSpeed--;
  }*/

  moveLeftWheel(dir, robotSpeed);
  moveRightWheel(dir, robotSpeed);
}


/**
   Stops motors of the robot
*/
void halt() {

  analogWrite(MOTOR_E1_PIN, 0);
  analogWrite(MOTOR_E2_PIN, 0);
  Serial.println("Halting");
}

/**
   Rotates the robot in place
   @param dir - the direction the robot is turning
                  true for right, false for left
   @param duration - the duration the robot is turning
*/
void rotate(boolean dir, int duration) {
  moveRightWheel(!dir, 255 * 0.5);
  moveLeftWheel(dir, 255 * 0.5);
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
  analogWrite(MOTOR_E1_PIN, robotSpeed);
}

/**
   Moves the left wheel in a direction with a speed
   @param dir - the direction the wheel is moving
                  true for forward, false for backward
   @param robotSpeed - the speed the wheel is moving
*/
void moveLeftWheel(boolean dir, int robotSpeed) {
  digitalWrite(MOTOR_M2_PIN, dir);
  analogWrite(MOTOR_E2_PIN, robotSpeed);
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


