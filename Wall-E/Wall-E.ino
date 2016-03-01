#define NOFIELD 100L
#define TOMILLIGAUSS 1953L
#include <LiquidCrystal.h>
#include <Servo.h>

//PIN DEFINITION
LiquidCrystal lcd(13, 12, 11, 10, 9, 8); //RS, EN, D4, D5, D6, D7
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

const int HALL_EFFECT_PIN = A5;
const int LEFT_OPTIC_PIN = A4;
const int RIGHT_OPTIC_PIN = A3;
const int TEMP_SENSOR_PIN = A2;
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
const int STUCK_THRESHOLD = 50;

int leftWhite = 0;
int rightWhite = 0;

// Variables will change:
int currentState = 0;   // counter for the number of button presses
int inputState = 0;         // current state of the button
int lastInputState = 0;     // previous state of the button
int isStopped = 0;

//Speed modelling variables
double A;
double B;

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
  pinMode(PUSH_BUTTON_PIN, INPUT);

  //Setting up the modelling variables
  A = MAX_SPEED / (pow(D_MAX, 2.0) - pow(D_MIN, 2.0));
  B = (MAX_SPEED * pow(D_MIN, 2.0) / (pow(D_MIN, 2.0) - pow(D_MAX, 2.0)));
}

void loop() {
  inputState = digitalRead(PUSH_BUTTON_PIN);

  if (inputState != lastInputState) {
    currentState = (currentState + 1) % 3;
    switch (currentState) {
      case 0: Serial.println("Autonomous Mode"); break;
      case 1: Serial.println("Line Follow Mode"); break;
      case 2: Serial.println("BT Controller Mode"); break;
      default: Serial.println("UNKNOWN MODE");
    }
    delay(50); //Debouncing
  }

  lastInputState = inputState;

  /*switch (currentState) {
    case 0: autonomous_loop(); break;
    case 1: line_follow_loop(); break;
    case 2: bluetooth_loop(); break;
    }*/
}

void autonomous_loop() {
  myservo.write(90);
  //get rid of interference
  long distance = max(max(get_distance(), get_distance()), get_distance());
  if ((distance < D_MAX ) && (D_STALL > abs(distance - get_distance()))) {
    isStopped++;
  }
  int robotSpeed = adjustSpeed(distance);
  moveInDirection(FORWARD, robotSpeed);
  Serial.println("Distance: " + String(distance));
  Serial.println("RobotSpeed: " + String(robotSpeed));
  if (distance < D_MIN || isStopped > STUCK_THRESHOLD) {
    if (isStopped > 50) {
      Serial.println("Robot Stuck detected at: " + String(isStopped));
    }
    Serial.println("Object closer than threshold detected");
    halt();
    int escapeAngle = findEscapeRoute();
    Serial.println("EA: " + String(escapeAngle));
    rotateAngle(escapeAngle);
    isStopped = 0;
  }
}

void line_follow_loop() {
  long gauss = get_gauss();
  if (gauss < 0) {
    Serial.println("Magnetic Field detected");
    lcd.print("Hello magnetic field");
  } else {
    Serial.println("Can't seem to detect field, should spin around");
    lcd.print("No field detected");
  }
  delay(500);
  lcd.clear();
}

void bluetooth_loop() {
  Serial.println("In bluetooth mode...");
  delay(10000);
}

/**
   Moves the robot in a specified direction at a specified speed
   @param dir - the direction the robot is heading
                  true for forward, false for backwards
   @param robotSpeed - the speed the robot is moving
*/
void moveInDirection(boolean dir, int robotSpeed) {
  moveLeftWheel(dir, robotSpeed * LEFT_MOTOR_ADJUSTMENT);
  moveRightWheel(dir, robotSpeed * RIGHT_MOTOR_ADJUSTMENT);
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

