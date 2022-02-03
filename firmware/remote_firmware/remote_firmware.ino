#include <radio.h>
#include "quad_remote.h"  // Header file with pin definitions and setup
#include <serLCD.h>
#include <EEPROM.h>
#include "transmission.h"


// CONSTANTS
const int THR_POS = 0;
const int YAW_POS = 8;
const int ROLL_POS = 16;
const int PIT_POS = 24;
const int AXIS_MIN = 0;
const int AXIS_MAX = 255;
const int SERIAL_BAUD = 9600;        // Baud rate for serial port


//Global Variables
int throttleRange[2];
int yawRange[2];
int rollRange[2];
int pitchRange[2];

response_pkt pkt;
  
bool calibrationActive = false;
bool quadcopterArmed = false;
int yaw = 0;
int throttle = 0;
int roll = 0;
int pitch = 0;

void btn1_pressed(bool);
void btn2_pressed(bool);
void set_gimbals();
void check_arm_status();
void print_gimbals();
void calibrateGimbals();
void print_range();


void setup() {\

  Serial.begin(SERIAL_BAUD);           // Start up serial
  delay(100);
  quad_remote_setup();
  
  rfBegin(RF_CHANNEL);
  Serial.print("Channel: ");
  Serial.println(RF_CHANNEL);

  btn1_cb = btn1_pressed;
  btn2_cb = btn2_pressed;
  lcd.setBacklight(0x000000FF);

  eeprom_load(THR_POS, throttleRange);
  eeprom_load(YAW_POS, yawRange);
  eeprom_load(ROLL_POS, rollRange);
  eeprom_load(PIT_POS, pitchRange);
}

void loop() {
  if(calibrationActive){
    calibrateGimbals();
  } else {
    if (millis() % 10 == 0) {  // Read gimbal values every 10ms
      set_gimbals();
    }
    if(millis() % 80 == 0){
      if(recieve_response(&pkt)){
        quadcopterArmed = pkt.armed;
      }
    }
    check_arm_status();
    if (millis() % 50 == 0) {  // Send a packet every 50ms
      send_packet(throttle, yaw, roll, pitch, quadcopterArmed);
    }

    //Serial.println("Response: ");
    //recieve_response();
  }
  if (millis() % 100 == 0) {
    print_gimbals();
  }
}

void calibrateGimbals() {

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

void print_range() {
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

void set_gimbals() {
  throttle = analogRead(PIN_THROTTLE);
  throttle = map(throttle, throttleRange[0], throttleRange[1], AXIS_MIN, AXIS_MAX);
  if (throttle <= 15) {
    throttle = 0;
  }
  yaw = analogRead(PIN_YAW);
  yaw = map(yaw, yawRange[0], yawRange[1], AXIS_MIN, AXIS_MAX);
  roll = analogRead(PIN_ROLL);
  roll = map(roll, rollRange[0], rollRange[1], AXIS_MIN, AXIS_MAX);
  pitch = analogRead(PIN_PITCH);
  pitch = map(pitch, pitchRange[0], pitchRange[1], AXIS_MIN, AXIS_MAX);
}

void print_gimbals() {
  if (quadcopterArmed) {
    Serial.print("A ");
  } else {
    Serial.print(". ");
  }
  Serial.print(throttle);
  Serial.print(" ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.println(pitch);
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

void btn2_pressed(bool down) {
  if (down && quadcopterArmed) {
    quadcopterArmed = false;
    lcd.setBacklight(0x000000FF);
  }
}

void check_arm_status() {
  if (!quadcopterArmed && throttle == 0 && yaw >= 250 && roll >= 250 && pitch >= 250) {
    quadcopterArmed = true;
    lcd.setBacklight(0x00FF0000);
  }
}
