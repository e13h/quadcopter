#include "radio.h"
#include "transmission.h"

const int MOTOR_1 = 8;
const int MOTOR_2 = 3;
const int MOTOR_3 = 4;
const int MOTOR_4 = 5;
const int MOTOR_SHUTOFF_TIMEOUT = 5000;  // milliseconds

bool armed = false;
int throttle = 0;
int yaw = 0;
int roll = 0;
int pitch = 0;
unsigned long time_last_good_pkt = 0;
quad_pkt pkt;

void handle_packet(quad_pkt);
void print_gimbals();

void setup() {
   const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port
   Serial.begin(SERIAL_BAUD);
   delay(100);
   rfBegin(RF_CHANNEL);
   pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (millis() % 50 == 0 && rfAvailable()) {
    if (recieve_packet(&pkt)) {
      handle_packet(pkt);
    }
  }

  if (millis() % 100 == 0) {
    print_gimbals();
  }

  if (millis() - time_last_good_pkt > MOTOR_SHUTOFF_TIMEOUT) {
    armed = false;
  }

  // Apply throttle to the motors
  if (armed) {
    digitalWrite(LED_BUILTIN, HIGH);
    analogWrite(MOTOR_1, throttle);
    analogWrite(MOTOR_2, throttle);
    analogWrite(MOTOR_3, throttle);
    analogWrite(MOTOR_4, throttle);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    analogWrite(MOTOR_1, 0);
    analogWrite(MOTOR_2, 0);
    analogWrite(MOTOR_3, 0);
    analogWrite(MOTOR_4, 0);
  }
}

void handle_packet(quad_pkt pkt) {
  time_last_good_pkt = millis();
  armed = pkt.armed;
  throttle = pkt.throttle;
  yaw = pkt.yaw;
  roll = pkt.roll;
  pitch = pkt.pitch;
}

void print_gimbals() {
  if (armed) {
    Serial.print("A ");
  } else {
    Serial.print(". ");
  }
  Serial.print(throttle);
  Serial.print(" ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.println(pitch);
}
