#include "radio.h"
#include "transmission.h"

const int AXIS_MIN = 0;
const int AXIS_MAX = 255;

void send_packet(int throttle, int yaw, int roll, int pitch, bool armed) {
  quad_pkt pkt;
  pkt.yaw = constrain(yaw, AXIS_MIN, AXIS_MAX);
  pkt.throttle = constrain(throttle, AXIS_MIN, AXIS_MAX);
  pkt.roll = constrain(roll, AXIS_MIN, AXIS_MAX);
  pkt.pitch = constrain(pitch, AXIS_MIN, AXIS_MAX);
  pkt.armed = armed;
  pkt.checksum = pkt.magic_constant ^ pkt.yaw ^ pkt.throttle ^ pkt.roll ^ pkt.pitch ^ pkt.armed;

  uint8_t* pkt_bytes = (uint8_t*) &pkt;
  rfWrite(pkt_bytes, sizeof(quad_pkt));
  // print_bytes(pkt_bytes, sizeof(quad_pkt));  // Debugging only
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
