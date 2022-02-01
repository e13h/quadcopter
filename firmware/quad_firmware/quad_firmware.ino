#include "radio.h"
#include "transmission.h"

void handle_packet(quad_pkt);

void setup() {
   const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port
   Serial.begin(SERIAL_BAUD);
   delay(100);
   rfBegin(RF_CHANNEL);
   pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  uint8_t buffer[sizeof(quad_pkt)];
  if (millis() % 50 == 0 && rfAvailable()) {
    rfRead(buffer, sizeof(quad_pkt));
    if (checksum_valid(buffer, sizeof(quad_pkt))) {
      quad_pkt pkt = *((quad_pkt*) buffer);
      handle_packet(pkt);
    } else {
      rfFlush();
    }
  }
  
}

void handle_packet(quad_pkt pkt) {
  if (pkt.armed) {
    Serial.print("A ");
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    Serial.print(". ");
    digitalWrite(LED_BUILTIN, LOW);
  }
 Serial.print(pkt.throttle);
 Serial.print(" ");
 Serial.print(pkt.yaw);
 Serial.print(" ");
 Serial.print(pkt.roll);
 Serial.print(" ");
 Serial.println(pkt.pitch);
}
