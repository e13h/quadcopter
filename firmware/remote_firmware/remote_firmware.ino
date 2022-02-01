#include <radio.h>
#include "quad_remote.h"  // Header file with pin definitions and setup
#include <serLCD.h>

struct quad_pkt {
  uint8_t magic_constant = 176;
  uint8_t yaw;
  uint8_t throttle;
  uint8_t roll;
  uint8_t pitch;
  bool armed;
  uint8_t checksum;
};

bool calibrationActive = false;
bool quadcopterArmed = false;
int yaw = 0;
int throttle = 0;
int roll = 0;
int pitch = 0;
const int AXIS_MIN = 0;
const int AXIS_MAX = 255;

void btn1_pressed(bool);

void setup() {
  const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port 
	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
	quad_remote_setup();

  btn1_cb = btn1_pressed;
}

void loop() {
  throttle = analogRead(PIN_THROTTLE);
  throttle = map(throttle, 0, 1023, AXIS_MIN, AXIS_MAX);

  yaw = analogRead(PIN_YAW);
  yaw = map(yaw, 0, 1023, AXIS_MIN, AXIS_MAX);
  
  roll = analogRead(PIN_ROLL);
  roll = map(roll, 0, 1023, AXIS_MIN, AXIS_MAX);

  pitch = analogRead(PIN_PITCH);
  pitch = map(pitch, 0, 1023, AXIS_MIN, AXIS_MAX);


  delay(1000);
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
