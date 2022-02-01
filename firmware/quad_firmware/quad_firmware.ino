#include "transmission.h"
#include "radio.h"

const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port
const int RF_CHANNEL = 15;

quad_pkt pkt;
int throttle;
int yaw;
int roll;
int pitch;
bool armed;

void setup() {
  // put your setup code here, to run once:
  throttle = 0;
  yaw = 0;
  roll = 0;
  pitch = 0;
  armed = false;

  rfBegin(RF_CHANNEL);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(recieve_packet(&pkt)){
    throttle = pkt.throttle;
    yaw = pkt.yaw;
    roll = pkt.roll;
    pitch = pkt.pitch;
    armed = pkt.armed;

    
  }
  if (millis() % 25 == 0) {
    send_response(armed, pkt.checksum);
  }
}

