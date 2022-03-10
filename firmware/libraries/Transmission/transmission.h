#ifndef TRANSMISSION
#define TRANSMISSION

#include "Arduino.h"

const int MAGIC_CONSTANT = 176;

struct pid_gains {
  float p_gain = 0.0;
  float i_gain = 0.0;
  float d_gain = 0.0;
};

struct quad_pkt {
  uint8_t magic_constant = MAGIC_CONSTANT;
  int8_t yaw;
  uint8_t throttle;
  int8_t roll;
  int8_t pitch;
  bool armed;
  uint8_t scaledCompFilterGain;
  uint8_t pitchScaledPGain;
  uint8_t pitchScaledIGain;
  uint8_t pitchScaledDGain;
  uint8_t rollScaledPGain;
  uint8_t rollScaledIGain;
  uint8_t rollScaledDGain;
  uint8_t yawScaledPGain;
  uint8_t yawScaledIGain;
  uint8_t yawScaledDGain;
  uint8_t checksum;
};

const int RF_CHANNEL = 15;

struct response_pkt {
  uint8_t magic_constant = MAGIC_CONSTANT;
  bool armed;
  uint8_t checksum;
  uint8_t response_CheckSum;
};


void send_packet(int, int, int, int, bool, float, pid_gains, pid_gains, pid_gains);
bool recieve_packet(quad_pkt&);
void send_response(bool,int);
bool recieve_response(response_pkt&);
void print_bytes(uint8_t*, uint8_t);
bool checksum_valid(uint8_t*, uint8_t);

#endif
