/**
   Determines if we are within range of an EMF
*/
/*
long get_gauss(bool isLeft) {
  long raw_field = analogRead(isLeft ? HALL_EFFECT_LEFT_PIN : HALL_EFFECT_RIGHT_PIN);
  long compensated = raw_field - NOFIELD;
  long gauss = compensated * TOMILLIGAUSS / 1000;

  return gauss;
}
*/
/**
   Get the distance in centimeters to the closest detected object
*/
unsigned long get_distance() {
  //delay(50);

  //Read the temperature and calculate the echo denominator, might calculate only once later
  //float temperature = (( analogRead(TEMP_SENSOR_PIN) / 1024.0) * 5000) / 10;
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
/* Turns the robot either left or right,
  depending on in which direction it can go further */
int findEscapeRoute() {
  int distance;
  int maxDistance = 0;
  int maxAngle = 0;
  
  for (int p = 0; p <= 180; p++) {
    myservo.write(p);

    distance = get_distance();
    if(distance > maxDistance){
      maxDistance = distance;
      maxAngle = p;
    }
    delay(10);
  }
  
  myservo.write(90);

  return maxAngle;
}

/*
void bluetooth() {
  // Forward
  if (bt_btn == '1') {
    int start = millis();
    int current = millis();
    moveInDirection(1, MAX_SPEED);
    while (current < start + 500)  {
      current = millis();
      if (current >= start + 500)
        halt();
    }
  }
  // Left
  if (bt_btn == '2') {
    int start = millis();
    int current = millis();
    rotateAngle(-135)
    while (current < start + 500)  {
      current = millis();
      if (current >= start + 500)
        halt();
    }
  }
  // Backward
  if (bt_btn == '3') {
    int start = millis();
    int current = millis();
    moveInDirection(0, MAX_SPEED);
    while (current < start + 500)  {
      current = millis();
      if (current >= start + 500)
        halt();
    }
  }
  // Right
  if (bt_btn == '4') {
    int start = millis();
    int current = millis();
    rotateAngle(45)
    while (current < start + 500)  {
      current = millis();
      if (current >= start + 500)
        halt();
    }
  } 
}
*/
