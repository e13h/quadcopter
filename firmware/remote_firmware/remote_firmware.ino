#include <radio.h>
#include "quad_remote.h"  // Header file with pin definitions and setup
#include <serLCD.h>
#include <EEPROM.h>

struct quad_pkt {
  uint8_t magic_constant = 176;
  uint8_t yaw;
  uint8_t throttle;
  uint8_t roll;
  uint8_t pitch;
  bool armed;
  uint8_t checksum;
};

// CONSTANTS
#define THR_POS 0
#define YAW_POS 8
#define ROLL_POS 16
#define PIT_POS 24
const int AXIS_MIN = 0;
const int AXIS_MAX = 255;


//Global Variables
int throttleRange[2];
int yawRange[2];
int rollRange[2];
int pitchRange[2];
  
bool calibrationActive = false;
bool quadcopterArmed = false;
int yaw = 0;
int throttle = 0;
int roll = 0;
int pitch = 0;

void btn1_pressed(bool);
void set_gimbals();
void print_gimbals();
void calibrateGimbals();
void print_range();
void send_packet();
void print_bytes(uint8_t*, uint8_t);
bool checksum_valid(uint8_t*, uint8_t);

void setup() {
  const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port
  const int RF_CHANNEL = 15;

	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
	quad_remote_setup();
  rfBegin(RF_CHANNEL);

  btn1_cb = btn1_pressed;

  eeprom_load(THR_POS, throttleRange);

  eeprom_load(YAW_POS, yawRange);

  eeprom_load(ROLL_POS, rollRange);
  
  eeprom_load(PIT_POS, pitchRange);
}

void loop() {
  if(calibrationActive){
    calibrateGimbals();
  } else {
    set_gimbals();
    send_packet();
    delay(1000);
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
  yaw = analogRead(PIN_YAW);
  yaw = map(yaw, yawRange[0], yawRange[1], AXIS_MIN, AXIS_MAX);
  roll = analogRead(PIN_ROLL);
  roll = map(roll, rollRange[0], rollRange[1], AXIS_MIN, AXIS_MAX);
  pitch = analogRead(PIN_PITCH);
  pitch = map(pitch, pitchRange[0], pitchRange[1], AXIS_MIN, AXIS_MAX);
}

void print_gimbals() {
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

void send_packet() {
  quad_pkt pkt;
  pkt.yaw = constrain(yaw, AXIS_MIN, AXIS_MAX);
  pkt.throttle = constrain(throttle, AXIS_MIN, AXIS_MAX);
  pkt.roll = constrain(roll, AXIS_MIN, AXIS_MAX);
  pkt.pitch = constrain(pitch, AXIS_MIN, AXIS_MAX);
  pkt.armed = quadcopterArmed;
  pkt.checksum = pkt.magic_constant ^ pkt.yaw ^ pkt.throttle ^ pkt.roll ^ pkt.pitch ^ pkt.armed;

  uint8_t* pkt_bytes = (uint8_t*) &pkt;
  // rfWrite(pkt_bytes, sizeof(quad_pkt));  // TODO: actually write the packet
  print_bytes(pkt_bytes, sizeof(quad_pkt));  // TODO: remove this line
}

void print_bytes(uint8_t* bytes, uint8_t len) {
  if (!checksum_valid(bytes, len)) {
    Serial.println("Packet integrity bad");
    return;
  }
  quad_pkt* pkt = (quad_pkt*) bytes;
  Serial.print("Yaw: ");
  Serial.print(pkt->yaw);
  Serial.print(" Throttle: ");
  Serial.print(pkt->throttle);
  Serial.print(" Roll: ");
  Serial.print(pkt->roll);
  Serial.print(" Pitch: ");
  Serial.print(pkt->pitch);
  Serial.print(" Armed: ");
  Serial.print(pkt->armed);
  Serial.print(" Checksum: ");
  Serial.println(pkt->checksum);
}

bool checksum_valid(uint8_t* bytes, uint8_t len) {
  if (len < sizeof(quad_pkt)) {
    // If the number of bytes doesn't match the size of the packet,
    // do not open the packet!
    return false;
  }
  uint8_t actual_checksum = 0;
  for (int i = 0; i < len - sizeof(uint8_t); i++) {
    actual_checksum ^= bytes[i];
  }
  uint8_t expected_checksum = bytes[len - sizeof(uint8_t)];
  return actual_checksum == expected_checksum;
}
