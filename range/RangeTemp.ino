#include <Servo.h>

Servo myservo;
const int trig = 6;
const int echo = 10;
const int temp = A1;
  float distanceleft;
  float distanceright;
  float distance;
  float temperature;
  float soundspeed;
void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  myservo.attach(9);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  myservo.write(0);
  distanceright = outputDistance();
  myservo.write(180);
  distanceleft = outputDistance();

  
  
}

float outputDistance() {
  
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  //following equation might need slight tweaking
  temperature = ((float) analogRead(temp) * 4.87/1024.0/0.01) - 273.15;
  soundspeed = 331.4 + 0.6 * temperature;
  distance = pulseIn(echo, HIGH) / (20000.0 / soundspeed);
  //Serial.print("Distance = ");
  //Serial.println(distance);
  //Serial.print("Temperature = ");
  //Serial.println(temperature);
  return distance;
}




