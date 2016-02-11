/*
  Arduino Hall Effect Sensor Project
  by Arvind Sanjeev
  Please check out  http://diyhacking.com for the tutorial of this project.
  DIY Hacking
*/


volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;
void magnet_detect() {
  half_revolutions++;
  Serial.println("detect");
}
void setup() {
  Serial.begin(9600);
  attachInterrupt(0, magnet_detect, RISING);//Initialize the intterrupt pin (Arduino digital pin 2)
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;
}

void loop(){
  if (half_revolutions >= 20) {
    rpm = 30 * 1000 / (millis() - timeold) * half_revolutions;
    timeold = millis();
    half_revolutions = 0;
    Serial.println(rpm,DEC);
  }
}

/*const int ULTRASONIC_ECHO_PIN = 11;
  const int ULTRASONIC_TRIG_PIN = 12;
  const int TEMP_SENSOR_PIN = A2;

  const int HALE_PIN = 5;
  const int REFLECT_OPTIC_PIN = A0;

  void setup() {
  Serial.begin(9600);
  pinMode(HALE_PIN, INPUT);
  }

  void loop() {
  float hal = digitalRead(HALE_PIN);
  Serial.println(hal);
  delay(250);
  }

  void ultrasonic_setup(){
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  }

  unsigned long ultrasonic_distance() {
  delay(50);

  //Read the temperature and calculate the echo denominator
  float temperature = (( analogRead(TEMP_SENSOR_PIN) / 1024.0) * 5000) / 10;
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
*/
