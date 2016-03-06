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

//Motor Driver Constants
const double LEFT_MOTOR_ADJUSTMENT = 0.97;
const double RIGHT_MOTOR_ADJUSTMENT = 1.0;
const double MAX_SPEED = 255;
const int TURN_TIME = 800; //how long it takes the robot to spin 90 deg

//CONSTANTS
const boolean RIGHT = true;
const boolean LEFT = false;
const boolean FORWARD = true;
const boolean BACK = false;

//LCD Constants
const int FIRST_COL = 0;
const int TOP_ROW = 1;
const int BOTTOM_ROW = 1;

//State changing
int currentState = 2;   // counter for the number of button presses

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

//--------------Bluetooth stuff
byte byteRead;
char const BT_HALT = '0';
char const BT_FORWARD = '1';
char const BT_BACK = '2';
char const BT_LEFT = '3';
char const BT_RIGHT = '4';
char const MODE_AUTO = 'a';
char const MODE_LINE = 'l';
char const MODE_BT = 'c';

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
  if (Serial.available()) {
    /* read the most recent byte */
    byteRead = Serial.read();
    Serial.println(char(byteRead));
    /*ECHO the value that was read, back to the serial port. */

    lcd.clear();
    switch (byteRead) {
      case MODE_AUTO:
        lcd.print("Autonomous Mode");
        currentState = 0;
        break;
      case MODE_LINE:
        lcd.print("Line Follow Mode");
        currentState = 1;
        break;
      case MODE_BT:
        lcd.print("Bluetooth Mode");
        currentState = 2;
        halt();
        break;
    }

  }


  switch (currentState) {
    case 0: autonomousLoop(); break;
    case 1: lineFollowLoop(); break;
    case 2: bluetooth_loop();  break;
  }

}

/**
   Loop for binary autonomous functionality
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
   Loop for angular autonomous pathfinding
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
   Loop for line following functionality
*/
void lineFollowLoop() {
  //Multiple reads to reduce interference
  analogRead(LEFT_OPTIC_PIN);
  leftSensor = analogRead(LEFT_OPTIC_PIN);
  analogRead(RIGHT_OPTIC_PIN);
  rightSensor = analogRead(RIGHT_OPTIC_PIN);

  if (leftSensor > leftWhite + LIGHT_THRESHOLD) {
    moveLeftWheel(FORWARD, 0);
    moveRightWheel(FORWARD, MAX_SPEED);
    robotState = TURN_LEFT;
  } else if (rightSensor > rightWhite + LIGHT_THRESHOLD) {
    // if right sensor detects path, move right
    moveRightWheel(FORWARD, 0);
    moveLeftWheel(FORWARD, MAX_SPEED);
    robotState = TURN_RIGHT;
  } else {
    // if nothing is detected, move straight
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

void bluetooth_loop() {
  switch (byteRead) {
    case BT_HALT: halt(); break;
    case BT_FORWARD: moveInDirection(FORWARD, MAX_SPEED); break;
    case BT_BACK: moveInDirection(BACK, MAX_SPEED); break;
    case BT_LEFT: rotateAngle(105); break;
    case BT_RIGHT: rotateAngle(85); break;
  }
}
