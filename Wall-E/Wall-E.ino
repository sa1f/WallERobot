#define NOFIELD 90L
#define TOMILLIGAUSS 1953L
#include <LiquidCrystal.h>
#include <Servo.h>

//PIN DEFINITION
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); //RS, EN, D4, D5, D6, D7

const int ULTRASONIC_TRIG_PIN = 3;
const int ULTRASONIC_ECHO_PIN = 2;
const int PUSH_BUTTON_PIN = 1;
//const int BLUETOOTH_PIN = 0;

const int LEFT_OPTIC_PIN = A3;
const int RIGHT_OPTIC_PIN = A2;
const int TEMP_SENSOR_PIN = A1;
Servo myservo; //SERVO WILL BE CONNECTED ON A0

const int MOTOR_E1_PIN = 5;
const int MOTOR_M1_PIN = 4;
const int MOTOR_E2_PIN = 6;
const int MOTOR_M2_PIN = 7;

const double LEFT_MOTOR_ADJUSTMENT = 0.94;
const double RIGHT_MOTOR_ADJUSTMENT = 1.0;
const double MAX_SPEED = 255;
const int TURN_TIME = 800; //how long it takes the robot to spin 90 deg

//CONSTANTS
const boolean RIGHT = true;
const boolean LEFT = false;
const boolean FORWARD = true;
const boolean BACK = false;

const int FIRST_COL = 0;
const int TOP_ROW = 1;
const int BOTTOM_ROW = 1;

//State changing
int currentState = 0;   // counter for the number of button presses
int inputState = 0;         // current state of the button
int lastInputState = -1;     // previous state of the button

//Autonomous Mode
const double D_MAX = 40;
const double D_MIN = 10;
const int D_STALL = 3;
const int STUCK_THRESHOLD = 60;

int isStopped = 0;
double temperature;

//Line Follow Mode
const int LIGHT_THRESHOLD = 200;
const int LINE_SPEED = 140;

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
long long _time = 0;
int _speed = 100;
long long _delay = 5000;

double A;
double B;


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
      case 2: lcd.print("Motor Test Mode"); break;
      default: lcd.print("UNKNOWN MODE");

    }
    delay(1000); //Debouncing
  }

  lastInputState = inputState;

  switch (currentState) {
    case 0: motorTestLoop(); break;
    case 1: lineFollowLoop(); break;
    case 2: autonomousLoop(); break;
  }

}

void autonomousLoop() {
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
  lcd.print("Speed Ratio: " + String(robotSpeed));
  if (distance < D_MIN || isStopped > STUCK_THRESHOLD) {
    lcd.clear();
    if (isStopped > 50) {
      lcd.print("Stuck at: " + String(distance));
    } else {
      lcd.clear();
      lcd.print("Object detected at:");
      lcd.setCursor(FIRST_COL, BOTTOM_ROW);
      lcd.print("Dist: " + String(distance));
    }
    halt();
    int escapeAngle = findEscapeRoute();
    lcd.setCursor(FIRST_COL, BOTTOM_ROW);
    lcd.print("EA: " + String(escapeAngle));
    rotateAngle(escapeAngle);
    isStopped = 0;
  }
}

void lineFollowLoop() {
  /*if (millis() > (_time + _delay)) {
    _speed = _speed + 10;
    if (_speed > 255) {
      _speed = 255;
    }
    _time = millis();
  }*/
  analogRead(LEFT_OPTIC_PIN);
  leftSensor = analogRead(LEFT_OPTIC_PIN);
  analogRead(RIGHT_OPTIC_PIN);
  rightSensor = analogRead(RIGHT_OPTIC_PIN);
  if (leftSensor > leftWhite + LIGHT_THRESHOLD) {
    moveLeftWheel(FORWARD, 0);
    moveRightWheel(FORWARD, LINE_SPEED);
    robotState = TURN_LEFT;
  }
  // if right sensor detects path, move right
  else if (rightSensor > rightWhite + LIGHT_THRESHOLD) {
    moveRightWheel(FORWARD, 0);
    moveLeftWheel(FORWARD, LINE_SPEED);
    robotState = TURN_RIGHT;
  }
  // if nothing is detected, move straight
  else {
    moveInDirection(FORWARD, LINE_SPEED);
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
    lastRobotState = robotState;
  }
}

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
