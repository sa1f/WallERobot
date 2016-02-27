#define NOFIELD 1023L
#define TOMILLIGAUSS 1953L
#include <LiquidCrystal.h>

//PIN DEFINITION
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

const int MOTOR_E1_PIN = 5;
const int MOTOR_M1_PIN = 4;
const int MOTOR_E2_PIN = 6;
const int MOTOR_M2_PIN = 7;

const int ULTRASONIC_ECHO_PIN = 2;
const int ULTRASONIC_TRIG_PIN = 3;

const int HALL_EFFECT_PIN = A5;
const int LEFT_OPTIC_PIN = A4;
const int RIGHT_OPTIC_PIN = A3;
const int TEMP_SENSOR_PIN = A2;

//CONSTANTS
const boolean RIGHT = true;
const boolean LEFT = false;
const boolean FORWARD = true;
const boolean BACK = false;

const int MAX_SPEED = 255;
const int TURN_TIME = 400; //how long it takes the robot to spin 90 deg

int leftWhite = 0;
int rightWhite = 0;

void setup() {
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  
  //Initialize value of the white background
  leftWhite = analogRead(A3);
  rightWhite = analogRead(A5);
  
  lcd.begin(16, 2);
  Serial.begin(9600);
  
  pinMode(MOTOR_M1_PIN, OUTPUT);
  pinMode(MOTOR_M2_PIN, OUTPUT);
}

void loop(){
  moveInDirection(FORWARD, MAX_SPEED * 0.5);
  delay(2000);
  turnLeft();
  turnLeft();
  delay(1000);

  int leftSensor = analogRead(LEFT_OPTIC_PIN);
  int rightSensor = analogRead(RIGHT_OPTIC_PIN);

  // if left sensor detects path, move left
  if (leftSensor > leftWhite) {
    Serial.println("Path detected on left sensor, turn left!");
    turnLeft();
  }else if (rightSensor > rightWhite) {
    Serial.println("Path detected on right sensor, turn right!");
    turnRight();
  } else {
    Serial.println("Moving along path");
    moveInDirection(FORWARD, MAX_SPEED * 0.5);
  }
  delay(500);
}

/**
   Moves the robot in a specified direction at a specified speed
   @param dir - the direction the robot is heading
                  true for forward, false for backwards
   @param robotSpeed - the speed the robot is moving
*/
void moveInDirection(boolean dir, int robotSpeed) {
  moveRightWheel(dir, robotSpeed);
  moveLeftWheel(dir, robotSpeed);
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
  rotate(LEFT, TURN_TIME);
}

/**
   Turns the robot right
*/
void turnRight() {
  rotate(RIGHT, TURN_TIME);
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
    rotate(LEFT, (angle - 90) / 90 * TURN_TIME);
  } else {
    rotate(RIGHT, angle / 90 * TURN_TIME);
  }
}

