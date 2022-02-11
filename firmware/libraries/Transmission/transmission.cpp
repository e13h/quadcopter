#include "radio.h"
#include "transmission.h"

const int AXIS_MIN = 0;
const int AXIS_MAX = 255;

void send_packet(int throttle, int yaw, int roll, int pitch, bool armed, float compFilterGain) {
  quad_pkt pkt;
  pkt.yaw = constrain(yaw, AXIS_MIN, AXIS_MAX);
  pkt.throttle = constrain(throttle, AXIS_MIN, AXIS_MAX);
  pkt.roll = constrain(roll, AXIS_MIN, AXIS_MAX);
  pkt.pitch = constrain(pitch, AXIS_MIN, AXIS_MAX);
  pkt.armed = armed;

  // Scale the filter gain so that we can easily compute the checksum
  pkt.scaledCompFilterGain = (int)(100.0 * constrain(compFilterGain, 0.0, 1.0));

  pkt.checksum = pkt.magic_constant ^ pkt.yaw ^ pkt.throttle ^ pkt.roll ^ pkt.pitch ^ pkt.armed ^ pkt.scaledCompFilterGain;

  uint8_t* pkt_bytes = (uint8_t*) &pkt;
  rfWrite(pkt_bytes, sizeof(quad_pkt));
  // print_bytes(pkt_bytes, sizeof(quad_pkt));  // Debugging only
}

bool recieve_packet(quad_pkt* q_pkt){
  uint8_t* pkt = (uint8_t*)q_pkt;
  
  if(rfAvailable()){
    rfRead(pkt,sizeof(quad_pkt));

    if(!checksum_valid(pkt, sizeof(quad_pkt))){
      rfFlush();
      return false;
    }
    return true;
  }
  else{
    return false;
  }
}

void send_response(bool armed, int checksum){
  response_pkt pkt;

  pkt.armed = armed;
  pkt.checksum = checksum;
  pkt.response_CheckSum = pkt.checksum ^ pkt.magic_constant ^ pkt.armed;

  uint8_t* pkt_bytes = (uint8_t*) &pkt;
  rfWrite(pkt_bytes, sizeof(response_pkt));
}

bool recieve_response(response_pkt* pkt){
  if(rfAvailable()){
    rfRead((uint8_t*)&pkt,sizeof(response_pkt));
    if(checksum_valid((uint8_t*)&pkt,sizeof(response_pkt))){
      return true;
    }
    else{
      rfFlush();
    }
  }
  else{
    Serial.println("no response...");
  }
  return false;
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
  Serial.print(" ScaledCompFilterGain: ");
  Serial.println(pkt->scaledCompFilterGain);
  Serial.print(" Checksum: ");
  Serial.println(pkt->checksum);
}

bool checksum_valid(uint8_t* bytes, uint8_t len) {
  uint8_t actual_checksum = 0;
  for (int i = 0; i < len - sizeof(uint8_t); i++) {
    actual_checksum ^= bytes[i];
  }
  uint8_t expected_checksum = bytes[len - sizeof(uint8_t)];
  return actual_checksum == expected_checksum;
}


