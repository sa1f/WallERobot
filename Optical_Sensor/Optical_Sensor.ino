int leftWhite;
int rightWhite;

void setup() {
Serial.begin(9600);

// initialize value of the white background
leftWhite = analogRead(A3);
rightWhite = analogRead(A5);
}

void loop() {
int leftSensor = analogRead(A3);
int rightSensor = analogRead(A5);

// if left sensor detects path, move left
if(leftSensor > leftWhite){
  Serial.println("Path detected on left sensor, turn left!");
  // add code to turn left
}

// if right sensor detects path, move right
else if(rightSensor > rightWhite){
  Serial.println("Path detected on right sensor, turn right!");
  // add code to turn right
}

// if nothing is detected, move straight
else{
  Serial.println("Moving along path");
  // add code to move straight
}

// only for testing on serial monitor
delay(500);
}

