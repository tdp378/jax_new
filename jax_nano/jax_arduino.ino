#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>

// Pins
#define BATT_PIN A0
#define RAIL_PIN A1
#define FSR_FL   A2 // Front Left
#define FSR_FR   A3 // Front Right
#define FSR_RL   A6 // Rear Left
#define FSR_RR   A7 // Rear Right
#define NEO_PIN  6

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_NeoPixel eyes(8, NEO_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60); 
  bno.begin();
  eyes.begin();
  setEyes(0, 255, 0); // Green: All Systems Go
}

void loop() {
  // 1. Telemetry Data
  sensors_event_t event;
  bno.getEvent(&event);
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  // 2. Read Sensors
  float b = (analogRead(BATT_PIN) * 5.0 / 1023.0) * 4.0;
  float r = (analogRead(RAIL_PIN) * 5.0 / 1023.0) * 2.0;
  
  // FSRs return 0-1023 (0 = no pressure, 1023 = full stomp)
  int fl = analogRead(FSR_FL);
  int fr = analogRead(FSR_FR);
  int rl = analogRead(FSR_RL);
  int rr = analogRead(FSR_RR);

  // 3. Send to Pi (Format: batt,rail,head,roll,pitch,cal,fl,fr,rl,rr)
  Serial.print(b);   Serial.print(",");
  Serial.print(r);   Serial.print(",");
  Serial.print(event.orientation.x); Serial.print(",");
  Serial.print(event.orientation.y); Serial.print(",");
  Serial.print(event.orientation.z); Serial.print(",");
  Serial.print(sys); Serial.print(",");
  Serial.print(fl);  Serial.print(",");
  Serial.print(fr);  Serial.print(",");
  Serial.print(rl);  Serial.print(",");
  Serial.println(rr);

  // 4. Command Listening (Eyes, Servos, E-Stop)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '<') setEyes(Serial.parseInt(), Serial.parseInt(), Serial.parseInt());
    else if (cmd == '[') {
      int c = Serial.parseInt(); int a = Serial.parseInt();
      pwm.setPWM(c, 0, map(a, 0, 180, 150, 600));
    }
    else if (cmd == '!') for(int i=0; i<16; i++) pwm.setPWM(i, 0, 4096);
  }
  delay(20); 
}

void setEyes(int r, int g, int b) {
  for(int i=0; i<8; i++) eyes.setPixelColor(i, eyes.Color(r,g,b));
  eyes.show();
}