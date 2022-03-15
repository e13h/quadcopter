#include "radio.h"
#include "transmission.h"


void send_packet(int throttle, int yaw, int roll, int pitch, bool armed,
    float compFilterGain, pid_gains pitch_pid_gains, pid_gains roll_pid_gains,
    pid_gains yaw_pid_gains) {
  quad_pkt pkt;
  pkt.yaw = constrain(yaw, -128, 127);
  pkt.throttle = constrain(throttle, 0, 255);
  pkt.roll = constrain(roll, -128, 127);
  pkt.pitch = constrain(pitch, -128, 127);
  pkt.armed = armed;

  // Scale the filter gain so that we can easily compute the checksum
  pkt.scaledCompFilterGain = (int)(100.0 * constrain(compFilterGain, 0.0, 2.56));
  pkt.pitchScaledPGain = (int)(100.0 * constrain(pitch_pid_gains.p_gain, 0.0, 2.56));
  pkt.pitchScaledIGain = (int)(100.0 * constrain(pitch_pid_gains.i_gain, 0.0, 2.56));
  pkt.pitchScaledDGain = (int)(100.0 * constrain(pitch_pid_gains.d_gain, 0.0, 2.56));
  pkt.rollScaledPGain = (int)(100.0 * constrain(roll_pid_gains.p_gain, 0.0, 2.56));
  pkt.rollScaledIGain = (int)(100.0 * constrain(roll_pid_gains.i_gain, 0.0, 2.56));
  pkt.rollScaledDGain = (int)(100.0 * constrain(roll_pid_gains.d_gain, 0.0, 2.56));
  pkt.yawScaledPGain = (int)(100.0 * constrain(yaw_pid_gains.p_gain, 0.0, 2.56));
  pkt.yawScaledIGain = (int)(100.0 * constrain(yaw_pid_gains.i_gain, 0.0, 2.56));
  pkt.yawScaledDGain = (int)(100.0 * constrain(yaw_pid_gains.d_gain, 0.0, 2.56));

  pkt.checksum = pkt.magic_constant ^ pkt.yaw ^ pkt.throttle ^ pkt.roll
    ^ pkt.pitch ^ pkt.armed ^ pkt.scaledCompFilterGain
    ^ pkt.pitchScaledPGain ^ pkt.pitchScaledIGain ^ pkt.pitchScaledDGain
    ^ pkt.rollScaledPGain ^ pkt.rollScaledIGain ^ pkt.rollScaledDGain
    ^ pkt.yawScaledPGain ^ pkt.yawScaledIGain ^ pkt.yawScaledDGain;

  uint8_t* pkt_bytes = (uint8_t*) &pkt;
  rfWrite(pkt_bytes, sizeof(quad_pkt));
  // print_bytes(pkt_bytes, sizeof(quad_pkt));  // Debugging only
}

void send_motors(bool armed, int motor_1, int motor_2, int motor_3, int motor_4) {
  motor_pkt pkt;
  pkt.armed = armed;
  pkt.motor_1 = constrain(motor_1, 0, 255);
  pkt.motor_2 = constrain(motor_2, 0, 255);
  pkt.motor_3 = constrain(motor_3, 0, 255);
  pkt.motor_4 = constrain(motor_4, 0, 255);
  pkt.checksum = pkt.magic_constant ^ pkt.armed ^ pkt.motor_1 ^ pkt.motor_2 ^ pkt.motor_3 ^ pkt.motor_4;
  uint8_t* pkt_bytes = (uint8_t*) &pkt;
  rfWrite(pkt_bytes, sizeof(motor_pkt));
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
  Serial.print(pkt->scaledCompFilterGain);
  Serial.print(" PitchScaledPGain: ");
  Serial.print(pkt->pitchScaledPGain);
  Serial.print(" PitchScaledIGain: ");
  Serial.print(pkt->pitchScaledIGain);
  Serial.print(" PitchScaledDGain: ");
  Serial.print(pkt->pitchScaledDGain);
  Serial.print(" RollScaledPGain: ");
  Serial.print(pkt->rollScaledPGain);
  Serial.print(" RollScaledIGain: ");
  Serial.print(pkt->rollScaledIGain);
  Serial.print(" RollScaledDGain: ");
  Serial.print(pkt->rollScaledDGain);
  Serial.print(" YawScaledPGain: ");
  Serial.print(pkt->yawScaledPGain);
  Serial.print(" YawScaledIGain: ");
  Serial.print(pkt->yawScaledIGain);
  Serial.print(" YawScaledDGain: ");
  Serial.print(pkt->yawScaledDGain);
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


