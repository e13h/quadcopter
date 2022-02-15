#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <Wire.h>

#include "radio.h"
#include "transmission.h"
// Flags and ids
const int MOTOR_1 = 8;
const int MOTOR_2 = 3;
const int MOTOR_3 = 4;
const int MOTOR_4 = 5;
const int MOTOR_SHUTOFF_TIMEOUT = 5000;  // milliseconds
const bool FLAG_PRINT_GIMBALS = false;
const bool FLAG_PRINT_IMU = false;
const bool FLAG_PRINT_PITCH = true;
const bool FLAG_PRINT_ROLL = false;
const bool FLAG_PRINT_YAW = false;
const bool FLAG_PRINT_PID = true;

// Current packet values
bool armed = false;
int throttle = 0;
int yaw = 0;
int roll = 0;
int pitch = 0;

// Packet recieve cariables
unsigned long time_last_good_pkt = 0;
quad_pkt pkt;

// IMU data variables
quad_data_t orientation;
unsigned long orientationTimestamp = 0;
float gyroDelta = 0.0;

// Gain and filtering variables
float compFilterGain = 0.9;
float pitchFiltered = 0.0;
float rollFiltered = 0.0;
float yawFiltered = 0.0;

float pGain = 0.0;
float iGain = 0.0;
float dGain = 0.0;

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // Create LSM9DS0 board instance.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), NULL, &lsm.getGyro());

void handle_packet(quad_pkt);
void print_stats(unsigned long);
void setupIMU();
void runCompFilter();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // Pause until serial console opens
  }

  rfBegin(RF_CHANNEL);
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialise the LSM9DS0 board.
  if (!lsm.begin()) {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS1 detected!"));
    while (1)
      ;
  }
  // Setup the sensor gain and integration time.
  setupIMU();

  // Tell the remote to disarm.
  for (int start = millis(); millis() - start < 3000;) {
    send_response(false, -1);
    delay(20);
  }
}

unsigned long last = millis();
void loop() {
  unsigned long now = millis();

  if (recieve_packet(&pkt)) {
    handle_packet(pkt);
  }

  if (millis() - time_last_good_pkt > MOTOR_SHUTOFF_TIMEOUT) {
    armed = false;
  }
  
  ahrs.getQuadOrientation(&orientation);  // Update quad orientation
  orientation.pitch_rate = -orientation.pitch_rate;  // inverting the pitch rate
  gyroDelta = float(now - orientationTimestamp) / 1000.0;  // Calculate time (sec) since last update
  orientationTimestamp = now;
  if (millis() % 10 == 0) {
    print_stats(now - last);
  }
  runCompFilter();

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

  last = now;
}

void handle_packet(quad_pkt pkt) {
  time_last_good_pkt = millis();
  armed = pkt.armed;
  throttle = pkt.throttle;
  yaw = pkt.yaw;
  roll = pkt.roll;
  pitch = pkt.pitch;
  compFilterGain = float(pkt.scaledCompFilterGain) / 100.0;
  pGain = float(pkt.scaledPGain) / 100.0;
  iGain = float(pkt.scaledIGain) / 100.0;
  dGain = float(pkt.scaledDGain) / 100.0;
}

void print_stats(unsigned long iterationTime) {
  /* The bits of text printed before each value show up as labels on the
   serial plotter for easier viewing.
  */
  Serial.print(iterationTime);
  Serial.print(F(" "));
  if (armed) {
    Serial.print(F("A "));
  } else {
    Serial.print(F(". "));
  }
  if (FLAG_PRINT_GIMBALS) {
    Serial.print(F("thr_gim:"));
    Serial.print(throttle);
    Serial.print(F(" "));

    Serial.print(F("yaw_gim:"));
    Serial.print(yaw);
    Serial.print(F(" "));

    Serial.print("rol_gim:");
    Serial.print(roll);
    Serial.print(F(" "));

    Serial.print(F("pit_gim:"));
    Serial.print(pitch);
    Serial.print(F(" "));
  }
  if (FLAG_PRINT_PID) {
    Serial.print(F("P:"));
    Serial.print(pGain);
    Serial.print(F(" "));

    Serial.print(F("I:"));
    Serial.print(iGain);
    Serial.print(F(" "));

    Serial.print(F("D:"));
    Serial.print(dGain);
    Serial.print(F(" "));
  }
  if (FLAG_PRINT_PITCH || FLAG_PRINT_ROLL || FLAG_PRINT_YAW) {
    Serial.print("gain:");
    Serial.print(compFilterGain);
    Serial.print(" ");
  }
  if (FLAG_PRINT_PITCH) {
    Serial.print("xl_pitch:");
    Serial.print(orientation.pitch);
    Serial.print(" ");

    Serial.print("gy_pitch:");
    Serial.print(orientation.pitch_rate);
    Serial.print(" ");

    Serial.print("f_pitch:");
    Serial.print(pitchFiltered);
    Serial.print(" ");
  }
  if (FLAG_PRINT_ROLL) {
    Serial.print("xl_roll:");
    Serial.print(orientation.roll);
    Serial.print(" ");

    Serial.print("gy_roll:");
    Serial.print(orientation.roll_rate);
    Serial.print(" ");

    Serial.print("f_roll:");
    Serial.print(rollFiltered);
    Serial.print(" ");
  }
  if (FLAG_PRINT_YAW) {
    Serial.print("gy_yaw:");
    Serial.print(orientation.yaw_rate);
    Serial.print(" ");

    Serial.print("f_yaw:");
    Serial.print(yawFiltered);
    Serial.print(" ");
  }
  Serial.println(F(""));
}

void setupIMU() {
  // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G,
             ODR_476 | G_BW_G_10);  // 952hz ODR + 63Hz cutoff

  // Set filters on the gyroscope (Section 7.13) to use just a high pass filter
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG2_G, G_OUTSEL_HP);
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_G, G_HP_EN | G_HP_CUT_1001);

  // Enable the XL (Section 7.23)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG5_XL,
             XL_ENABLE_X | XL_ENABLE_Y | XL_ENABLE_Z);

  // Set low-pass XL filter frequency divider (Section 7.25)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL,
             HR_MODE | XL_LP_ODR_RATIO_400);

  // This only sets range of measurable values for each sensor.  Setting these
  // manually (I.e., without using these functions) will cause incorrect output
  // from the library.
  lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_2000DPS);
}

void runCompFilter() {
  // Serial.println(gyroDelta);
  pitchFiltered = compFilterGain * (pitchFiltered + (gyroDelta * orientation.pitch_rate))
    + (1 - compFilterGain) * orientation.pitch;
}
