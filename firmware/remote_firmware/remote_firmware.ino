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

  send_packet();
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
