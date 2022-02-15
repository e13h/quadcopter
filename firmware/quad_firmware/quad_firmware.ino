#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <Wire.h>

#include "radio.h"
#include "transmission.h"

// Flags and ids
const int MOTOR_1 = 3;
const int MOTOR_2 = 4;
const int MOTOR_3 = 5;
const int MOTOR_4 = 8;
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
float loopDeltaTime = 0.0;

// Gain and filtering variables
float compFilterGain = 0.9;
float pitchFiltered = 0.0;
float rollFiltered = 0.0;
float yawFiltered = 0.0;

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // Create LSM9DS0 board instance.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), NULL, &lsm.getGyro());

// PID variables
float pGain = 0.0;
float iGain = 0.0;
float dGain = 0.0;

float prevPitchErr = 0;
float sumPitchErr = 0;

float prev_time = 0;
float cur_time = 0;
float pid_pitch = 0.0;


void handle_packet(quad_pkt);
void print_stats(unsigned long);
void setupIMU();
void runCompFilter();
float PID_calc(float, float, float, float);
void mixer();

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
  loopDeltaTime = float(now - orientationTimestamp) / 1000.0;  // Calculate time (sec) since last update
  orientationTimestamp = now;
  if (millis() % 10 == 0) {
    print_stats(now - last);
  }
  runCompFilter();

  // Apply throttle to the motors
  mixer();

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

    Serial.print(F("pid_pitch:"));
    Serial.print(pid_pitch);
    Serial.print(F(" "));
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
  pitchFiltered = compFilterGain * (pitchFiltered + (loopDeltaTime * orientation.pitch_rate))
    + (1 - compFilterGain) * orientation.pitch;
}

float PID_calc(float prev_err, float cur_err, float delta_time, float integ_sum){
  float deriv_err = (cur_err - prev_err) / delta_time;

  if(throttle != 0){
    integ_sum =  (.75) * integ_sum  + .5 * (cur_err + prev_err) * delta_time; 
  }
  
  prev_err = cur_err;

  return pGain * cur_err + dGain * deriv_err + iGain * integ_sum;
}

void mixer() {
  deadband();

  int motor_1_throttle = throttle;
  int motor_2_throttle = throttle;
  int motor_3_throttle = throttle;
  int motor_4_throttle = throttle;

  pid_pitch = PID_calc(prevPitchErr, pitchFiltered, loopDeltaTime, sumPitchErr);

  motor_1_throttle -= pid_pitch;
  motor_2_throttle -= pid_pitch;
  motor_3_throttle += pid_pitch;
  motor_4_throttle += pid_pitch;

  if (armed) {
    digitalWrite(LED_BUILTIN, HIGH);
    analogWrite(MOTOR_1, motor_1_throttle);
    analogWrite(MOTOR_2, motor_2_throttle);
    analogWrite(MOTOR_3, motor_3_throttle);
    analogWrite(MOTOR_4, motor_4_throttle);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    analogWrite(MOTOR_1, 0);
    analogWrite(MOTOR_2, 0);
    analogWrite(MOTOR_3, 0);
    analogWrite(MOTOR_4, 0);
  }
}

void deadband() {
  const uint8_t THROTTLE_DEADBAND = 15;
  const uint8_t PITCH_DEADBAND = 15;
  const uint8_t ROLL_DEADBAND = 15;
  const uint8_t YAW_DEADBAND = 15;
  const uint8_t CENTERING_ORIGIN = 128;

  if (throttle < THROTTLE_DEADBAND) {
    throttle = 0;
  }
  if (pitch >= CENTERING_ORIGIN - PITCH_DEADBAND && pitch <= CENTERING_ORIGIN + PITCH_DEADBAND) {
    pitch = CENTERING_ORIGIN;
  }
  if (roll >= CENTERING_ORIGIN - ROLL_DEADBAND && roll <= CENTERING_ORIGIN + ROLL_DEADBAND) {
    roll = CENTERING_ORIGIN;
  }
  if (yaw >= CENTERING_ORIGIN - YAW_DEADBAND && yaw <= CENTERING_ORIGIN + YAW_DEADBAND) {
    yaw = CENTERING_ORIGIN;
  }
}
