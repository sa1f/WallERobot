#include <LiquidCrystal.h>
#include <Servo.h>

//--------PIN DEFINITION--------
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); //RS, EN, D4, D5, D6, D7

const int ULTRASONIC_TRIG_PIN = 3;
const int ULTRASONIC_ECHO_PIN = 2;
//BLUETOOTH ON PINS 0 AND 1 FOR 2-WAY SERIAL COMMUNICATION

const int LEFT_OPTIC_PIN = A3;
const int RIGHT_OPTIC_PIN = A2;
const int TEMP_SENSOR_PIN = A1;
Servo myservo; //SERVO WILL BE CONNECTED ON A0

const int MOTOR_E1_PIN = 5;
const int MOTOR_M1_PIN = 4;
const int MOTOR_E2_PIN = 6;
const int MOTOR_M2_PIN = 7;

//---------Motor Driver Constants---------
const double LEFT_MOTOR_ADJUSTMENT = 0.97;
const double RIGHT_MOTOR_ADJUSTMENT = 1.0;
const double MAX_SPEED = 255;
const int TURN_TIME = 800;

//Motor Direction Constants
const boolean RIGHT = true;
const boolean LEFT = false;
const boolean FORWARD = true;
const boolean BACK = false;

//Speed Modelling for Motor Acceleration
double A;
double B;

//--------LCD Constants--------
const int FIRST_COL = 0;
const int TOP_ROW = 1;
const int BOTTOM_ROW = 1;

//----Functionality State Changing----
int currentState = 2;   //Start with Bluetooth mode by default

//--------Autonomous Mode--------
const double D_MAX = 40; //Distance robot slows down
const double D_MIN = 10; //Distance robot should stop

//Variables for checking whether robot is stuck
const int D_STALL = 3;
const int STUCK_THRESHOLD = 60;
int isStopped = 0;

//For Ultrasonic calculation
double temperature;

//--------Line Follow Mode--------
const int LIGHT_THRESHOLD = 200;
const int LINE_SPEED = 150;

int leftWhite = 0;
int rightWhite = 0;
int leftSensor;
int rightSensor;

//Robot States for Line Follow Mode
int robotState;
int lastRobotState = 0;
const int FULL_SPEED = 0;
const int TURN_LEFT = 1;
const int TURN_RIGHT = 2;

//--------------Bluetooth-----------
byte byteRead;
char const BT_HALT = '0';
char const BT_FORWARD = '1';
char const BT_BACK = '2';
char const BT_LEFT = '3';
char const BT_RIGHT = '4';
char const BT_WIGGLE_WIGGLE = 'w';
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

  //Setting up the modelling variables
  A = MAX_SPEED / (pow(D_MAX, 2.0) - pow(D_MIN, 2.0));
  B = (MAX_SPEED * pow(D_MIN, 2.0) / (pow(D_MIN, 2.0) - pow(D_MAX, 2.0)));

  //Temperature initialization
  temperature = (( analogRead(TEMP_SENSOR_PIN) / 1024.0) * 5000) / 10;

  // Initialize serial for bluetooth communication
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    //Read from Bluetooth to determine state change
    byteRead = Serial.read();
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
    case 0: autonomousLoop(false); break;
    case 1: lineFollowLoop(); break;
    case 2: bluetooth_loop();  break;
  }

}

/**
   Perform Autonomous navigation functionality
*/
void autonomousLoop(boolean useAngular) {
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
    int escapeAngle = (useAngular ? findEscapeAngle() : findEscapeRoute());

    lcd.setCursor(FIRST_COL, BOTTOM_ROW);
    lcd.print("Turn: " + String(escapeAngle));
    rotateAngle(escapeAngle);
    isStopped = 0;
  }
}

/**
   Perform Line Following functionality
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

/**
  Perform bluetooth control functionality; commands from
  the bluetooth daemon will be sent as a char byte depending
  on the required actions.
*/
void bluetooth_loop() {
  switch (byteRead) {
    case BT_HALT:
      Serial.println("Halting");
      halt();
      break;

    case BT_FORWARD:
      Serial.println("Going forward");
      moveInDirection(FORWARD, MAX_SPEED);
      break;

    case BT_BACK:
      Serial.println("Going backward");
      moveInDirection(BACK, MAX_SPEED);
      break;

    case BT_LEFT:
      Serial.println("Turning left");
      rotateAngle(105);
      break;

    case BT_RIGHT:
      Serial.println("Turning right");
      rotateAngle(85);
      break;

    case BT_WIGGLE_WIGGLE:
      Serial.println("Wiggle in progress");
      lcd.clear();
      lcd.print("Wiggle in");
      lcd.setCursor(FIRST_COL, BOTTOM_ROW);
      lcd.print("progress");

      for (int i = 0; i < 2; i++) {
        wiggle(3);
        moveInDirection(FORWARD, MAX_SPEED);
        delay(1500);
        turnLeft();
        turnLeft();
      }

      wiggle(5);
      lcd.clear();
      lcd.print("Wiggle complete!");
      Serial.println("Wiggle complete!");
      delay(2000);

      byteRead = '0';
      break;
  }
}

/**
   Make the robot do a little dance; to be used in
   bluetooth mode for more additional "features"
*/
void wiggle(int amount) {
  for (int i = 0; i < amount; i++) {
    rotateAngle(45);
    rotateAngle(135);
  }
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
   Get the distance in centimeters to the closest detected object
   from the ultrasonic sensor
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
  Turns the robot strictly left or right,
  depending on in which direction it can go further
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
  if (get_distance() > distance) {
    maxAngle = 180;
  }
  myservo.write(90);
  return maxAngle;
}

/**
  Turns the robot in any angle
  depending on in which direction it can go further
*/
int findEscapeAngle() {
  int distance;
  int maxDistance = 0;
  int maxAngle = 0;

  for (int p = 0; p <= 180; p++) {
    myservo.write(p);
    delay(10);
    if (p == 0) {
      delay(20); //Timing issue in initial servo pan?
    }
    distance = get_distance();
    if (distance > maxDistance) {
      maxDistance = distance;
      maxAngle = p;
    }

  }
  return maxAngle;
}
