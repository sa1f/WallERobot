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
int MAX_SPEED = 255;

//Constants
int turnTime = 400;

void setup() 
{ 
    pinMode(M1, OUTPUT);   
    pinMode(M2, OUTPUT); 
} 
 
void loop() 
{ 
  rotate(LEFT);
  delay(1000);
  rotate(RIGHT);
  delay(1000);
}

/**
 * Moves the robot in a specified direction at a specified speed
 * @param dir - the direction the robot is heading
 *                true for forward, false for backwards
 * @param robotSpeed - the speed the robot is moving
 */
void moveInDirection(boolean dir, int robotSpeed){
  digitalWrite(M1, dir);
  digitalWrite(M2, dir);
  analogWrite(E1, robotSpeed);
  analogWrite(E2, robotSpeed);
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
 */
void rotate(boolean dir){
  digitalWrite(M1, dir);
  digitalWrite(M2, !dir);
  analogWrite(E1, MAX_SPEED);
  analogWrite(E2, MAX_SPEED);
  delay(turnTime);
  halt();
}

