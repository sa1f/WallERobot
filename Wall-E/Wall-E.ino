const int ULTRASONIC_ECHO_PIN = 11;
const int ULTRASONIC_TRIG_PIN = 12;
const int TEMP_SENSOR_PIN = A2;

void setup() {
  Serial.begin(9600);
}

void loop() {

  delay(250);
}

void ultrasonic_setup() {
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

