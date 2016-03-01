/**
 * Movement driver for the robot using PWM as speed variable
 */

//PIN Definitions
int E1 = 5;  
int M1 = 4; 
int E2 = 6;                      
int M2 = 7;                        

//Keywords
boolean RIGHT = true;
boolean LEFT = false;
boolean FORWARD = true;
boolean BACK = false;
int MAX_SPEED = 110;
boolean state = false;
//Constants
int turnTime = 100; //how long it takes the robot to spin 90 deg;

int leftWhite;
int rightWhite;

int leftSensor;
int rightSensor;

void setup() 
{ 
  pinMode(M1, OUTPUT);   
  pinMode(M2, OUTPUT);
  leftWhite = analogRead(A1);
  rightWhite = analogRead(A0);
 // Serial.begin(9600); 
} 
 
void loop() 
{
  /*if (Serial.available() > 0){
    if (Serial.read() == 's'){
      state != state;
    }
  }
  if (state){*/
    moveAlongPath();
  /*}
  else{
    halt();
  }
  */
}

/**
 * Follows a black path
 */
void moveAlongPath(){
  //analogRead(A1);
  leftSensor = analogRead(A1);
  //analogRead(A0);
  rightSensor = analogRead(A0);
  //Serial.print("Left Sensor: ");
  //Serial.println(leftSensor);
  //Serial.print("Right Sensor: ");
  //Serial.println(rightSensor);
  if(leftSensor > leftWhite + 50){
    moveLeftWheel(FORWARD, 0);
    //Serial.println("turning left");
  }
  
  // if right sensor detects path, move right
  else if(rightSensor > rightWhite + 50){
    moveRightWheel(FORWARD, 0);
    //Serial.println("turning right");
  }
  
  // if nothing is detected, move straight
 else{
    moveInDirection(FORWARD, MAX_SPEED);
    //Serial.println("Full Speed");  
  }
}

/**
 * Moves the robot in a specified direction at a specified speed
 * @param dir - the direction the robot is heading
 *                true for forward, false for backwards
 * @param robotSpeed - the speed the robot is moving
 */
void moveInDirection(boolean dir, int robotSpeed){
  moveRightWheel(dir, robotSpeed);
  moveLeftWheel(dir, robotSpeed);
}


/**
 * Stops motors of the robot
 */
void halt(){
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}

/**
 * Rotates the robot in place 
 * @param dir - the direction the robot is turning
 *                true for right, false for left
 * @param duration - the duration the robot is turning
 */
void rotate(boolean dir, int duration){
  moveRightWheel(!dir, MAX_SPEED*0.5);
  moveLeftWheel(dir, MAX_SPEED*0.5);
  delay(duration);
  halt();
}

/**
 * Turns the robot left
 */
void turnLeft(){
  rotate(LEFT, turnTime);
}

/**
 * Turns the robot right
 */
void turnRight(){
  rotate(RIGHT, turnTime);
}



/**
 * Moves the right wheel in a direction with a speed
 * @param dir - the direction the wheel is moving
 *                true for forward, false for backward
 * @param robotSpeed - the speed the wheel is moving
 */
void moveRightWheel(boolean dir, int robotSpeed){
  digitalWrite(M1, dir);
  analogWrite(E1, robotSpeed);
}


/**
 * Moves the left wheel in a direction with a speed
 * @param dir - the direction the wheel is moving
 *                true for forward, false for backward
 * @param robotSpeed - the speed the wheel is moving
 */
void moveLeftWheel(boolean dir, int robotSpeed){
  digitalWrite(M2, dir);
  analogWrite(E2, robotSpeed);
}

/**
 * Rotates the robot a specific angle in a direction. 
 * @Param angle - the angle it turns to 
 *		- 180 for left, 0 for right
 */
void rotateAngle(int angle){
	if (angle < 0 || angle > 180) return;
	if (angle > 90){
		rotate(LEFT, (angle-90)/90 * turnTime);
	} else {
		rotate(RIGHT, angle/90 * turnTime);
	}
}



