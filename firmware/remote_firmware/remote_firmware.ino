#include <radio.h>
#include "quad_remote.h"  // Header file with pin definitions and setup
#include <serLCD.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int throttle = analogRead(PIN_THROTTLE);
  throttle = map(throttle, 0, 1023, 0, 255);
  Serial.print(throttle);
  Serial.print(" ");

  int yaw = analogRead(PIN_YAW);
  yaw = map(yaw, 0, 1023, 0, 255);
  Serial.print(yaw);
  Serial.print(" ");
  
  int roll = analogRead(PIN_ROLL);
  roll = map(roll, 0, 1023, 0, 255);
  Serial.print(roll);
  Serial.print(" ");

  int pitch = analogRead(PIN_PITCH);
  pitch = map(pitch, 0, 1023, 0, 255);
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print("\n");
  delay(10);
}
