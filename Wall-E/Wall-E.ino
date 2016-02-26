#define NOFIELD 1023L
#define TOMILLIGAUSS 1953L

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int HALL_EFFECT_PIN = A5;

const int LEFT_OPTIC_PIN = A3;
const int RIGHT_OPTIC_PIN = A4;

const int ULTRASONIC_ECHO_PIN = 11;
const int ULTRASONIC_TRIG_PIN = 12;
const int TEMP_SENSOR_PIN = A2;

int leftWhite = 0;
int rightWhite = 0;

//PIN Definitions
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

//Constants
const boolean RIGHT = true;
const boolean LEFT = false;
const boolean FORWARD = true;
const boolean BACK = false;
const int MAX_SPEED = 255;
const int TURN_TIME = 400; //how long it takes the robot to spin 90 deg

//RIGHT WHEEL M1
//LEFT WHEEL M2

void setup() {
  setup_ultrasonic();
  setup_optics();
  
  //lcd.begin(16, 2);
  Serial.begin(9600);
  
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
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
  analogWrite(E1, 0);
  analogWrite(E2, 0);
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
  digitalWrite(M1, dir);
  analogWrite(E1, robotSpeed);
}


/**
   Moves the left wheel in a direction with a speed
   @param dir - the direction the wheel is moving
                  true for forward, false for backward
   @param robotSpeed - the speed the wheel is moving
*/
void moveLeftWheel(boolean dir, int robotSpeed) {
  digitalWrite(M2, dir);
  analogWrite(E2, robotSpeed);
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

