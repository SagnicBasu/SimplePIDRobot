//Code by Sagnic Basu from The Technovator youtube channel on youtube. Subscribe today!
#include <QTRSensors.h>
QTRSensors qtr;
// Line Sensor Properties

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];  

// PID Properties
const double KP = 0.1;
const double KD = 0.18;
double lastError = 0;
const int GOAL = 3500;
const int MAX_SPEED = 220;
const int BASE_SPEED = 200;
 
 
void setup() {
  
qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 7,8 }, SensorCount);
  qtr.setEmitterPin(2);

  // Initialize line sensor array
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.begin(9600);
}
 
void loop() {
 
  // Get line position
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  delay(20);
  // Compute error from line
  int error = GOAL - (position);
 
  // Compute motor adjustment
  int adjustment = KP*error + KD*(error - lastError);
 
  // Store error for next increment
  lastError = error;
 
  // Adjust motors 
   analogWrite(10,(constrain(BASE_SPEED - adjustment, 0, MAX_SPEED)));
   analogWrite(11,(constrain(BASE_SPEED + adjustment, 0, MAX_SPEED)));
 
}
 
