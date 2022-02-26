#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <Wire.h>

#include "radio.h"
#include "transmission.h"


struct axis {
  int gimbal_cmd = 0;
  int offset_degrees = 0;
  float pid = 0.0;
  float filtered = 0.0;

};

struct mixer_input_config {
  // Parameters for the mixer
  int gimbal_throttle = 0;
  axis yaw;
  axis roll;
  axis pitch;

  int motor1_throttle = 0;
  int motor2_throttle = 0;
  int motor3_throttle = 0;
  int motor4_throttle = 0;
};

struct pid_input_config {
  // Parameters for the PID controller
  float p_gain = 0.0;
  float i_gain = 0.0;
  float d_gain = 0.0;
  float prev_error = 0.0;
  float sum_error = 0.0;
};

// Flags and ids
const int MOTOR_1 = 3;
const int MOTOR_2 = 4;
const int MOTOR_3 = 5;
const int MOTOR_4 = 8;
const int MOTOR_SHUTOFF_TIMEOUT = 5000;  // milliseconds
const bool FLAG_PRINT_GIMBALS = false;
const bool FLAG_PRINT_IMU = false;
const bool FLAG_PRINT_PITCH = false;
const bool FLAG_PRINT_ROLL = false;
const bool FLAG_PRINT_YAW = false;
const bool FLAG_PRINT_PID = true;
const bool FLAG_PRINT_MOTORS = true;

// Packet recieve cariables
quad_pkt pkt_from_remote;
unsigned long pkt_from_remote_timestamp = 0;

// IMU data variables
quad_data_t orientation;
unsigned long orientationTimestamp = 0;
float loopDeltaTime = 0.0;
quad_data_t imu_offsets;
bool imu_calibrated = false;

// Gain and filtering variables
mixer_input_config mixer_inputs;

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // Create LSM9DS0 board instance.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), NULL, &lsm.getGyro());

// PID variables
pid_input_config pitch_pid_inputs;
pid_input_config roll_pid_inputs;
pid_input_config yaw_pid_inputs;



void print_stats(unsigned long);
void setupIMU();
void runCompFilter();
float PID_calc(pid_input_config&, float, float);
void mixer();
void deadband();
void offset();
void assign_PID_gains();
void calibrateIMU();
void applyIMUCalibration();

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

  if (recieve_packet(pkt_from_remote)) {
    pkt_from_remote_timestamp = millis();
    assign_PID_gains();
  }

  if (millis() - pkt_from_remote_timestamp > MOTOR_SHUTOFF_TIMEOUT) {
    pkt_from_remote.armed = false;
  }
  
  ahrs.getQuadOrientation(&orientation);  // Update quad orientation
  applyIMUCalibration();
  loopDeltaTime = float(millis() - orientationTimestamp) / 1000.0;  // Calculate time (sec) since last update
  orientationTimestamp = millis();
  if (millis() % 10 == 0) {
    print_stats(now - last);
  }

  if (pkt_from_remote.armed && !imu_calibrated) {
    // The first time that we arm the quad, calibrate the IMU.
    calibrateIMU();
    imu_calibrated = true;
  }

  if (pkt_from_remote.armed) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  mixer();

  last = now;
}

void print_stats(unsigned long iterationTime) {
  /* The bits of text printed before each value show up as labels on the
   serial plotter for easier viewing.
  */
  Serial.print(iterationTime);
  Serial.print(F(" "));
  if (pkt_from_remote.armed) {
    Serial.print(F("A "));
  } else {
    Serial.print(F(". "));
  }
  if (FLAG_PRINT_GIMBALS) {
    Serial.print(F("thr_gim:"));
    Serial.print(pkt_from_remote.throttle);
    Serial.print(F(" "));

    Serial.print(F("yaw_gim:"));
    Serial.print(pkt_from_remote.yaw);
    Serial.print(F(" "));

    Serial.print(F("rol_gim:"));
    Serial.print(pkt_from_remote.roll);
    Serial.print(F(" "));

    Serial.print(F("pit_gim:"));
    Serial.print(pkt_from_remote.pitch);
    Serial.print(F(" "));
  }
  if (FLAG_PRINT_PID) {
    if (FLAG_PRINT_PITCH) {
      Serial.print(F("P:"));
      Serial.print(pitch_pid_inputs.p_gain);
      Serial.print(F(" "));

      Serial.print(F("I:"));
      Serial.print(pitch_pid_inputs.i_gain);
      Serial.print(F(" "));

      Serial.print(F("D:"));
      Serial.print(pitch_pid_inputs.d_gain);
      Serial.print(F(" "));
    }
    if (FLAG_PRINT_ROLL) {
      Serial.print(F("P:"));
      Serial.print(roll_pid_inputs.p_gain);
      Serial.print(F(" "));

      Serial.print(F("I:"));
      Serial.print(roll_pid_inputs.i_gain);
      Serial.print(F(" "));

      Serial.print(F("D:"));
      Serial.print(roll_pid_inputs.d_gain);
      Serial.print(F(" "));
    }
    if (FLAG_PRINT_YAW) {
      Serial.print(F("P:"));
      Serial.print(yaw_pid_inputs.p_gain);
      Serial.print(F(" "));

      Serial.print(F("I:"));
      Serial.print(yaw_pid_inputs.i_gain);
      Serial.print(F(" "));

      Serial.print(F("D:"));
      Serial.print(yaw_pid_inputs.d_gain);
      Serial.print(F(" "));
    }
  }
  if (FLAG_PRINT_PITCH || FLAG_PRINT_ROLL || FLAG_PRINT_YAW) {
    Serial.print(F("gain:"));
    Serial.print(pkt_from_remote.scaledCompFilterGain / 100.0);
    Serial.print(F(" "));
  }
  if (FLAG_PRINT_IMU && FLAG_PRINT_PITCH) {
    Serial.print(F("xl_pitch:"));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));

    Serial.print(F("gy_pitch:"));
    Serial.print(orientation.pitch_rate);
    Serial.print(F(" "));

    Serial.print(F("f_pitch:"));
    Serial.print(mixer_inputs.pitch.filtered);
    Serial.print(F(" "));

    Serial.print(F("pid_pitch:"));
    Serial.print(mixer_inputs.pitch.pid);
    Serial.print(F(" "));

    Serial.print(F("off_pitch:"));
    Serial.print(mixer_inputs.pitch.offset_degrees);
    Serial.print(F(" "));
  }
  if (FLAG_PRINT_IMU && FLAG_PRINT_ROLL) {
    Serial.print(F("xl_roll:"));
    Serial.print(orientation.roll);
    Serial.print(F(" "));

    Serial.print(F("gy_roll:"));
    Serial.print(orientation.roll_rate);
    Serial.print(F(" "));

    Serial.print(F("f_roll:"));
    Serial.print(mixer_inputs.roll.filtered);
    Serial.print(F(" "));

    Serial.print(F("pid_roll:"));
    Serial.print(mixer_inputs.roll.pid);
    Serial.print(F(" "));

    Serial.print(F("off_roll:"));
    Serial.print(mixer_inputs.roll.offset_degrees);
    Serial.print(F(" "));
  }
  if (FLAG_PRINT_IMU && FLAG_PRINT_YAW) {
    Serial.print(F("gy_yaw:"));
    Serial.print(orientation.yaw_rate);
    Serial.print(F(" "));

    Serial.print(F("pid_yaw:"));
    Serial.print(mixer_inputs.yaw.pid);
    Serial.print(F(" "));

    Serial.print(F("off_yaw:"));
    Serial.print(mixer_inputs.yaw.offset_degrees);
    Serial.print(F(" "));
  }
  if (FLAG_PRINT_MOTORS) {
    Serial.print(F("m1:"));
    Serial.print(mixer_inputs.motor1_throttle);
    Serial.print(F(" "));

    Serial.print(F("m2:"));
    Serial.print(mixer_inputs.motor2_throttle);
    Serial.print(F(" "));

    Serial.print(F("m3:"));
    Serial.print(mixer_inputs.motor3_throttle);
    Serial.print(F(" "));

    Serial.print(F("m4:"));
    Serial.print(mixer_inputs.motor4_throttle);
    Serial.print(F(" "));
  }
  Serial.println(F(""));
}

void setupIMU() {
  // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G,
             ODR_238 | G_BW_G_10);  // 952hz ODR + 63Hz cutoff

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
  float compFilterGain = pkt_from_remote.scaledCompFilterGain / 100.0;
  mixer_inputs.pitch.filtered =
    compFilterGain * (mixer_inputs.pitch.filtered + (loopDeltaTime * orientation.pitch_rate))
    + (1 - compFilterGain) * orientation.pitch;
  mixer_inputs.roll.filtered =
    compFilterGain * (mixer_inputs.roll.filtered + (loopDeltaTime * orientation.roll_rate))
    + (1 - compFilterGain) * orientation.roll;
}

void assign_PID_gains() {
  pitch_pid_inputs.p_gain = pkt_from_remote.pitchScaledPGain / 100.0;
  pitch_pid_inputs.i_gain = pkt_from_remote.pitchScaledIGain / 100.0;
  pitch_pid_inputs.d_gain = pkt_from_remote.pitchScaledDGain / 100.0;
  roll_pid_inputs.p_gain = pkt_from_remote.rollScaledPGain / 100.0;
  roll_pid_inputs.i_gain = pkt_from_remote.rollScaledIGain / 100.0;
  roll_pid_inputs.d_gain = pkt_from_remote.rollScaledDGain / 100.0;
  yaw_pid_inputs.p_gain = pkt_from_remote.yawScaledPGain / 100.0;
  yaw_pid_inputs.i_gain = pkt_from_remote.yawScaledIGain / 100.0;
  yaw_pid_inputs.d_gain = pkt_from_remote.yawScaledDGain / 100.0;
}

float PID_calc(pid_input_config& config, float cur_err, float delta_time){
  float deriv_err = (cur_err - config.prev_error) / delta_time;

  if (pkt_from_remote.throttle == 0) {
    config.sum_error = 0;
  } else {
    config.sum_error = config.sum_error + .5 * (cur_err + config.prev_error) * delta_time; 
  }
  
  config.prev_error = cur_err;

  return config.p_gain * cur_err + config.d_gain * deriv_err + config.i_gain * config.sum_error;
}

void mixer() {
  // Copy gimbal commands from packet so that if we receive a new packet while
  // in this function, we don't get unexpected behavior. 
  mixer_inputs.gimbal_throttle = pkt_from_remote.throttle;
  mixer_inputs.pitch.gimbal_cmd = pkt_from_remote.pitch;
  mixer_inputs.roll.gimbal_cmd = pkt_from_remote.roll;
  mixer_inputs.yaw.gimbal_cmd = pkt_from_remote.yaw;

  deadband();
  offset();
  runCompFilter();
  
  // PID
  float pitch_error = mixer_inputs.pitch.offset_degrees - mixer_inputs.pitch.filtered;
  mixer_inputs.pitch.pid = PID_calc(pitch_pid_inputs, pitch_error, loopDeltaTime);
  float yaw_error = mixer_inputs.yaw.offset_degrees - orientation.yaw_rate;
  mixer_inputs.yaw.pid = PID_calc(yaw_pid_inputs, yaw_error, loopDeltaTime);


  // Mix
  mixer_inputs.motor1_throttle = mixer_inputs.gimbal_throttle 
    + mixer_inputs.pitch.offset_degrees + mixer_inputs.pitch.pid
    - mixer_inputs.yaw.offset_degrees + mixer_inputs.yaw.pid
    ;

  mixer_inputs.motor2_throttle = mixer_inputs.gimbal_throttle 
    + mixer_inputs.pitch.offset_degrees + mixer_inputs.pitch.pid
    + mixer_inputs.yaw.offset_degrees - mixer_inputs.yaw.pid
    ;

  mixer_inputs.motor3_throttle = mixer_inputs.gimbal_throttle 
    - mixer_inputs.pitch.offset_degrees - mixer_inputs.pitch.pid
    - mixer_inputs.yaw.offset_degrees + mixer_inputs.yaw.pid
    ;

  mixer_inputs.motor4_throttle = mixer_inputs.gimbal_throttle 
    - mixer_inputs.pitch.offset_degrees - mixer_inputs.pitch.pid
    + mixer_inputs.yaw.offset_degrees - mixer_inputs.yaw.pid
    ;

  int max_thr = max(max(
        max(mixer_inputs.motor4_throttle,mixer_inputs.motor3_throttle),
      mixer_inputs.motor2_throttle),
    mixer_inputs.motor1_throttle);

  int capper = 0;
  if(max_thr > 255){
    capper = max_thr-255;
  }

  // Constrain
  mixer_inputs.motor1_throttle = constrain(mixer_inputs.motor1_throttle-capper, 0, 255);
  mixer_inputs.motor2_throttle = constrain(mixer_inputs.motor2_throttle-capper, 0, 255);
  mixer_inputs.motor3_throttle = constrain(mixer_inputs.motor3_throttle-capper, 0, 255);
  mixer_inputs.motor4_throttle = constrain(mixer_inputs.motor4_throttle-capper, 0, 255);

  // Send commands to motors
  if (pkt_from_remote.armed && mixer_inputs.gimbal_throttle > 0) {
    analogWrite(MOTOR_1, mixer_inputs.motor1_throttle);
    analogWrite(MOTOR_2, mixer_inputs.motor2_throttle);
    analogWrite(MOTOR_3, mixer_inputs.motor3_throttle);
    analogWrite(MOTOR_4, mixer_inputs.motor4_throttle);
  } else {
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

  if (mixer_inputs.gimbal_throttle < THROTTLE_DEADBAND) {
    mixer_inputs.gimbal_throttle = 0;
  }
  if (mixer_inputs.pitch.gimbal_cmd >= CENTERING_ORIGIN - PITCH_DEADBAND
      && mixer_inputs.pitch.gimbal_cmd <= CENTERING_ORIGIN + PITCH_DEADBAND) {
    mixer_inputs.pitch.gimbal_cmd = CENTERING_ORIGIN;
  }
  if (mixer_inputs.roll.gimbal_cmd >= CENTERING_ORIGIN - ROLL_DEADBAND
      && mixer_inputs.roll.gimbal_cmd <= CENTERING_ORIGIN + ROLL_DEADBAND) {
    mixer_inputs.roll.gimbal_cmd = CENTERING_ORIGIN;
  }
  if (mixer_inputs.yaw.gimbal_cmd >= CENTERING_ORIGIN - YAW_DEADBAND
      && mixer_inputs.yaw.gimbal_cmd <= CENTERING_ORIGIN + YAW_DEADBAND) {
    mixer_inputs.yaw.gimbal_cmd = CENTERING_ORIGIN;
  }
}

void offset() {
  const int YAW_OFFSET_ANGLE_MAX = 180;  // degrees
  const int TILT_OFFSET_ANGLE_MAX = 10;  // degrees

  mixer_inputs.pitch.offset_degrees = map(mixer_inputs.pitch.gimbal_cmd, 0, 255, -TILT_OFFSET_ANGLE_MAX, TILT_OFFSET_ANGLE_MAX);
  mixer_inputs.roll.offset_degrees = map(mixer_inputs.roll.gimbal_cmd, 0, 255, -TILT_OFFSET_ANGLE_MAX, TILT_OFFSET_ANGLE_MAX);
  mixer_inputs.yaw.offset_degrees = map(mixer_inputs.yaw.gimbal_cmd, 0, 255, -YAW_OFFSET_ANGLE_MAX, YAW_OFFSET_ANGLE_MAX);
}

void calibrateIMU() {
  const int NUM_MEASUREMENTS = 20;
  quad_data_t measurements[NUM_MEASUREMENTS];
  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    ahrs.getQuadOrientation(&measurements[i]);
    delay(1);
  }
  imu_offsets.pitch = 0.0;
  imu_offsets.roll = 0.0;
  imu_offsets.pitch_rate = 0.0;
  imu_offsets.roll_rate = 0.0;
  imu_offsets.yaw_rate = 0.0;
  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    imu_offsets.pitch += measurements[i].pitch;
    imu_offsets.roll += measurements[i].roll;
    imu_offsets.pitch_rate += measurements[i].pitch_rate;
    imu_offsets.roll_rate += measurements[i].roll_rate;
    imu_offsets.yaw_rate += measurements[i].yaw_rate;
  }
  imu_offsets.pitch /= NUM_MEASUREMENTS;
  imu_offsets.roll /= NUM_MEASUREMENTS;
  imu_offsets.pitch_rate /= NUM_MEASUREMENTS;
  imu_offsets.roll_rate /= NUM_MEASUREMENTS;
  imu_offsets.yaw_rate /= NUM_MEASUREMENTS;

  imu_offsets.pitch_rate = -imu_offsets.pitch_rate;  // invert the pitch rate
}

void applyIMUCalibration() {
  orientation.pitch_rate = -orientation.pitch_rate;  // invert the pitch rate

  if (!imu_calibrated) {
    return;
  }
  orientation.pitch -= imu_offsets.pitch;
  orientation.roll -= imu_offsets.roll;
  orientation.pitch_rate -= imu_offsets.pitch_rate;
  orientation.roll_rate -= imu_offsets.roll_rate;
  orientation.yaw_rate -= imu_offsets.yaw_rate;
}
