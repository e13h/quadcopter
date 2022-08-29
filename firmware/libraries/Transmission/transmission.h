#ifndef TRANSMISSION
#define TRANSMISSION

#include "Arduino.h"

const int MAGIC_CONSTANT = 25;

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

struct motor_pkt {
  uint8_t magic_constant = MAGIC_CONSTANT;
  bool armed;
  uint8_t motor_1;
  uint8_t motor_2;
  uint8_t motor_3;
  uint8_t motor_4;
  uint8_t checksum;
};

void send_packet(int, int, int, int, bool, float, pid_gains, pid_gains, pid_gains);
void send_motors(bool, int, int, int, int);
void print_bytes(uint8_t*, uint8_t);
bool checksum_valid(uint8_t*, uint8_t);

template<class T>
bool receive_packet(T& pkt) {
  T buffer;
  
  if(rfAvailable()){
    rfRead((uint8_t*)&buffer, sizeof(T));
    if(!checksum_valid((uint8_t*)&buffer, sizeof(T)) || buffer.magic_constant != MAGIC_CONSTANT) {
      rfFlush();
      return false;
    }
    pkt = buffer;
    return true;
  } else {
    return false;
  }
}

#endif
