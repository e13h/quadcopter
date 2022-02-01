#include <radio.h>
#include "quad_remote.h"  // Header file with pin definitions and setup
#include <serLCD.h>
#include <EEPROM.h>

// CONSTANTS
#define THR_POS 0
#define YAW_POS 8
#define ROLL_POS 16
#define PIT_POS 24


//Global Variables
int throttleRange[2];
int yawRange[2];
int rollRange[2];
int pitchRange[2];
  
bool calibrationActive = false;
bool quadcopterArmed = false;

void btn1_pressed(bool);
void setup() {
  const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port 
	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
	quad_remote_setup();

  btn1_cb = btn1_pressed;

  eeprom_load(THR_POS, throttleRange);

  eeprom_load(YAW_POS, yawRange);

  eeprom_load(ROLL_POS, rollRange);
  
  eeprom_load(PIT_POS, pitchRange);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(calibrationActive){
    calibrateGimbals();
  }
}

void calibrateGimbals(){

  print_range();

  int curThrottle = analogRead(PIN_THROTTLE);
  int curYaw = analogRead(PIN_YAW);
  int curRoll = analogRead(PIN_ROLL);
  int curPitch = analogRead(PIN_PITCH);

  //Set default Min range
  throttleRange[0] = curThrottle;
  yawRange[0] = curYaw;
  rollRange[0] = curRoll;
  pitchRange[0] = curPitch;

  //Set default Max range
  throttleRange[1] = curThrottle;
  yawRange[1] = curYaw;
  rollRange[1] = curRoll;
  pitchRange[1] = curPitch;
  delay(500);

  while(calibrationActive){
    
    //Get new gimbal values
    curThrottle = analogRead(PIN_THROTTLE);
    curYaw = analogRead(PIN_YAW);
    curRoll = analogRead(PIN_ROLL);
    curPitch = analogRead(PIN_PITCH);

    // Update range if need be
    if(curThrottle < throttleRange[0]){
      throttleRange[0] = curThrottle;
    }
    if(curThrottle > throttleRange[1]){
      throttleRange[1] = curThrottle;
    }

    if(curYaw < yawRange[0]){
      yawRange[0] = curYaw;
    }
    if(curYaw > yawRange[1]){
      yawRange[1] = curYaw;
    }
    
    if(curRoll < rollRange[0]){
      rollRange[0] = curRoll;
    }
    if(curRoll > rollRange[1]){
      rollRange[1] = curRoll;
    }

    if(curPitch < pitchRange[0]){
      pitchRange[0] = curPitch;
    }
    if(curPitch > pitchRange[1]){
      pitchRange[1] = curPitch;
    }
  }

  // Display clalibration close
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Closing Calibration");

  // Update roll over values
  eeprom_store(THR_POS, throttleRange);

  eeprom_store(YAW_POS, yawRange);

  eeprom_store(ROLL_POS, rollRange);
  
  eeprom_store(PIT_POS, pitchRange);

  delay(1000);
  lcd.clear();
  
  // Debug
  print_range();
  return;
}

void print_range(){
  Serial.print("\nThrottle Min ");
  Serial.print(throttleRange[0]);
  Serial.print("\nThrottle Max ");
  Serial.print(throttleRange[1]);
  
  Serial.print("\nYaw Min ");
  Serial.print(yawRange[0]);
  Serial.print("\nYaw Max ");
  Serial.print(yawRange[1]);

  Serial.print("\nRoll Min ");
  Serial.print(rollRange[0]);
  Serial.print("\nRoll Max ");
  Serial.print(rollRange[1]);
  
  Serial.print("\nPitch Min ");
  Serial.print(pitchRange[0]);
  Serial.print("\nPitch Max ");
  Serial.print(pitchRange[1]);
}

void print_gimbals(){
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
