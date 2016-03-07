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
  if (get_distance() > distance){
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
    if(p == 0){
      delay(20); //Timing issue in initial servo pan?
    }
    distance = get_distance();
    if(distance > maxDistance){
      maxDistance = distance;
      maxAngle = p;
    }
  
  }
  return maxAngle;
}
