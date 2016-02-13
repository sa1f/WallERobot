/**
 * Movement driver for the robot using PWM as speed variable
 */

//PIN Definitions
int E1 = 5;  
int M1 = 4; 
int E2 = 6;                      
int M2 = 7;                        

//Keywords
boolean LEFT = false;
boolean RIGHT = true;

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

void moveInDirection(int dir, int robotSpeed){
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, robotSpeed);
  analogWrite(E2, robotSpeed);
}

void halt(){
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}

void rotate(boolean dir){
  digitalWrite(M1, dir);
  digitalWrite(M2, !dir);
  analogWrite(E1, 255);
  analogWrite(E2, 255);
  delay(turnTime);
  halt();
}

