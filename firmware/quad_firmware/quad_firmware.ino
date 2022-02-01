#include "transmission.h"
#include "radio.h"

const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port
const int RF_CHANNEL = 15;

quad_pkt pkt;

void setup() {
  // put your setup code here, to run once:
  

  rfBegin(RF_CHANNEL);
}

void loop() {
  // put your main code here, to run repeatedly:

}

