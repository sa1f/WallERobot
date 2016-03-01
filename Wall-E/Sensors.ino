/**
   Determines if we are within range of an EMF
*/
long get_gauss() {
  long raw_field = analogRead(HALL_EFFECT_PIN);
  long compensated = raw_field - NOFIELD;
  long gauss = compensated * TOMILLIGAUSS / 1000;
  Serial.println("Gauss: " + String(gauss));

  return gauss;
}

/**
   Get the distance in centimeters to the closest detected object
*/
unsigned long get_distance() {
  //delay(50);

  //Read the temperature and calculate the echo denominator, might calculate only once later
  float temperature = (( analogRead(TEMP_SENSOR_PIN) / 1024.0) * 5000) / 10;
  float speed_of_sound = 331.5 + (0.6 * temperature);
  float denom = (20000.0 / speed_of_sound);
  //Serial.println("Temp: "+String(temperature));
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
  
  //Serial.print("Max Disance: "+String(maxDistance));
  //Serial.println(" @"+String(maxAngle));
  
  /*if(maxAngle >= 100){
    Serial.println("Should turn left");
  }else if (maxAngle < 80){
    Serial.println("Should turn right");
  }else{
    Serial.println("Should move straight");
  }*/
  return maxAngle;
}

