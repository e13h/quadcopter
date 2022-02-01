#ifndef TRANSMISSION
#define TRANSMISSION

#include "Arduino.h"

const int MAGIC_CONSTANT = 176;

struct quad_pkt {
  uint8_t magic_constant = MAGIC_CONSTANT;
  uint8_t yaw;
  uint8_t throttle;
  uint8_t roll;
  uint8_t pitch;
  bool armed;
  uint8_t checksum;
};

struct response_pkt {
  uint8_t magic_constant = MAGIC_CONSTANT;
  bool armed;
  uint8_t checksum;
  uint8_t response_CheckSum;
};


void send_packet(int, int, int, int, bool);
bool recieve_packet(quad_pkt*);
void send_response(bool,int);
void recieve_response();
void print_bytes(uint8_t*, uint8_t);
bool checksum_valid(uint8_t*, uint8_t);
bool response_checksum_valid(uint8_t*, uint8_t);

#endif
