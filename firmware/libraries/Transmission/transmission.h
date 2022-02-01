#ifndef TRANSMISSION
#define TRANSMISSION

#include "Arduino.h"

struct quad_pkt {
  uint8_t magic_constant = 176;
  uint8_t yaw;
  uint8_t throttle;
  uint8_t roll;
  uint8_t pitch;
  bool armed;
  uint8_t checksum;
};

void send_packet(int, int, int, int, bool);
bool recieve_packet(quad_pkt*);
void print_bytes(uint8_t*, uint8_t);
bool checksum_valid(uint8_t*, uint8_t);

#endif
