#include <radio.h>
#include "quad_remote.h"  // Header file with pin definitions and setup
#include <serLCD.h>

bool calibrationActive = false;
bool quadcopterArmed = false;

void btn1_pressed(bool);

void setup() {
  const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port 
	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
	quad_remote_setup();

  btn1_cb = btn1_pressed;
}

void loop() {
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

void btn1_pressed(bool down) {
  if (down && !quadcopterArmed && !calibrationActive) {
    calibrationActive = !calibrationActive;

    // Print calibrating message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Calibrating");
  } else if (down && !quadcopterArmed && calibrationActive) {
    calibrationActive = !calibrationActive;

    // Print default message?
    lcd.clear();
  } else if (down && quadcopterArmed) {
    // Print message to disarm the quadcopter?
  }
}
