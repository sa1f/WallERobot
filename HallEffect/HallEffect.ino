#define NOFIELD 1023L    // Analog output with no applied field, calibrate this
#define TOMILLIGAUSS 1953L  // For A1301: 2.5mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 1953mG

void setup() {
  Serial.begin(9600);
}

void DoMeasurement() {
  int raw = analogRead(A5);

  long compensated = raw - NOFIELD;                 // adjust relative to no applied field
  long gauss = compensated * TOMILLIGAUSS / 1000;   // adjust scale to Gauss

  Serial.print(gauss);
  Serial.println(" Gauss");
}

void loop(){
  delay(1000);
  DoMeasurement();
}
