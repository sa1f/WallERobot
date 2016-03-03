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

//Motor Driver Constants
const double LEFT_MOTOR_ADJUSTMENT = 0.94;
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
double A;
double B;

//--------------Bluetooth stuff
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
    case 0: angularAutonomousLoop(); break;
    case 1: lineFollowLoop(); break;
    case 2: autonomousLoop(); break;
  }

}

/**
 * Loop for binary autonomous functionality
 */
void autonomousLoop() {
  myservo.write(90);
  //get rid of interference
  long distance = max(max(get_distance(), get_distance()), get_distance());
  //long distance = get_distance();
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
 */
void angularAutonomousLoop() {
  myservo.write(90);
  //get rid of interference
  long distance = max(max(get_distance(), get_distance()), get_distance());
  //long distance = get_distance();
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
