#define NOFIELD 100L
#define TOMILLIGAUSS 1953L
#include <LiquidCrystal.h>
#include <Servo.h>

Servo myservo;
//PIN DEFINITION
//RS, EN, D4,D5,D6,D7
LiquidCrystal lcd(4, 5, 8, 9, 10, 11);

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

const double MAX_SPEED = 255;
const int TURN_TIME = 1000; //how long it takes the robot to spin 90 deg

const double D_MAX = 30;
const double D_MIN = 10;

int leftWhite = 0;
int rightWhite = 0;

// Variables will change:
int currentState = 0;   // counter for the number of button presses
int inputState = 0;         // current state of the button
int lastInputState = 0;     // previous state of the button

//Speed modelling variable
int A;
int B;

void setup() {
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  myservo.attach(A0);

  /*//Initialize value of the white background
    leftWhite = analogRead(A3);
    rightWhite = analogRead(A5);
  */

  pinMode(MOTOR_M1_PIN, OUTPUT);
  pinMode(MOTOR_M2_PIN, OUTPUT);
  Serial.begin(9600);

  //lcd.begin(16, 2);
  //pinMode(7, INPUT);
  A = pow(MAX_SPEED, 2.0) / (pow(D_MAX, 2.0) - pow(D_MIN, 2.0));
  B = (MAX_SPEED * pow(D_MIN, 2.0) / (pow(D_MIN, 2.0) - pow(D_MAX, 2.0)));
}

void loop() {
  myservo.write(90);
  long distance = get_distance();
  moveInDirection(FORWARD, adjustSpeed(distance));
  //Serial.println(distance);
  if (distance < 10) {
    halt();
    int escapeAngle = findEscapeRoute();
    Serial.println("EA: " + String(escapeAngle));
    rotateAngle(escapeAngle);
  }


  /*
    long gauss = get_gauss();
    if(gauss < 0){
    Serial.println("Magnetic Field detected");
    lcd.print("Hello magnetic field");
    }else{
    Serial.println("Can't seem to detect field, should spin around");
    lcd.print("No field detected");
    }
    delay(500);
    lcd.clear();

  */
  /*inputState = digitalRead(7);

    if (inputState != lastInputState) {
    currentState = (currentState + 1) % 3;
    delay(50); //Debouncing
    }

    lastInputState = inputState;

    switch(currentState){
    case 0: Serial.println("Autonomous Mode"); break;
    case 1: Serial.println("Line Follow Mode"); break;
    case 2: Serial.println("BT Controller Mode"); break;
    default: Serial.println("UNKNOWN");
    }
    delay(500);
  */
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
    robotSpeed = A * distance ^ 2 + B;
  }
  return robotSpeed;
}

