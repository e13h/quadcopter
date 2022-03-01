#include <EEPROM.h>
#include <radio.h>
#include <SerLCD.h>

#include "quad_remote.h"  // Header file with pin definitions and setup
#include "transmission.h"

// CONSTANTS
const int THR_POS = 0;
const int YAW_POS = 12;
const int ROLL_POS = 24;
const int PIT_POS = 36;

const int COMP_FILTER_POS = 48;

const int PITCH_PID_P_POS = 52;
const int PITCH_PID_I_POS = 56;
const int PITCH_PID_D_POS = 60;

const int ROLL_PID_P_POS = 64;
const int ROLL_PID_I_POS = 68;
const int ROLL_PID_D_POS = 72;

const int YAW_PID_P_POS = 76;
const int YAW_PID_I_POS = 80;
const int YAW_PID_D_POS = 84;

const int PITCH_TRIM = 88;
const int ROLL_TRIM = 92;
const int YAW_TRIM = 96;

const int AXIS_MIN = 0;
const int AXIS_MAX = 255;

enum TuningParam {
  NONE,
  COMPLEMENTARY_FILTER,
  PROPORTIONAL,
  INTEGRAL,
  DERIVATIVE
};

enum TuningAxis {
  PITCH,
  ROLL,
  YAW
};

const char* TUNING_AXIS_LABELS[] = {"Pitch: ", "Roll: ", "Yaw: "};
const char* TUNING_PARAM_LABELS[] = {"", "Filter: ", "P-", "I-", "D-"};


// Global Variables
int throttleRange[3];
int yawRange[3];
int rollRange[3];
int pitchRange[3];

response_pkt pkt;

bool calibrationActive = false;
bool quadcopterArmed = false;
bool trimmingActive = false;

bool tuningActive = false;
TuningParam currentTuningParamType = NONE;
float* currentTuningParam = NULL;
TuningAxis currentTuningAxis = PITCH;

int yaw = 0;
int yaw_offset = 0;
int throttle = 0;
int roll = 0;
int roll_offset = 0;
int pitch = 0;
int pitch_offset = 0;

float complementaryFilterGain = 0.98;
pid_gains pitch_pid_gains;
pid_gains roll_pid_gains;
pid_gains yaw_pid_gains;
int pitch_trim = 0;
int roll_trim = 0;
int yaw_trim = 0;

// Function Declarations
void btn1_pressed(bool);
void btn2_pressed(bool);
void knob_pressed(bool);
void btn_down_pressed(bool);
void btn_left_pressed(bool);
void btn_up_pressed(bool);
void btn_right_pressed(bool);
void btn_center_pressed(bool);
void knobs_update();
void set_gimbals();
void check_arm_status();
void print_gimbals();
void calibrateGimbals();
void print_range();
void print_pid();
void begin_tuning();
void check_if_eeprom_loaded_nan(float&);
void updateLCD();
void deadband();
void offset();


// Implementation
void setup() {
  Serial.begin(115200);  // Start up serial
  delay(100);
  Serial.println("Remote is online!");
  pinMode(LED_BUILTIN, OUTPUT);
  quad_remote_setup();
  rfBegin(RF_CHANNEL);
  Serial.print("Channel: ");
  Serial.println(RF_CHANNEL);

  btn1_cb = btn1_pressed;
  btn2_cb = btn2_pressed;
  knob1_btn_cb = knob_pressed;
  btn_down_cb = btn_down_pressed;
  btn_left_cb = btn_left_pressed;
  btn_up_cb = btn_up_pressed;
  btn_right_cb = btn_right_pressed;
  btn_center_cb = btn_center_pressed;
  knobs_update_cb = knobs_update;

  eeprom_load(THR_POS, throttleRange);
  eeprom_load(YAW_POS, yawRange);
  eeprom_load(ROLL_POS, rollRange);
  eeprom_load(PIT_POS, pitchRange);
  eeprom_load(COMP_FILTER_POS, complementaryFilterGain);
  check_if_eeprom_loaded_nan(complementaryFilterGain);
  eeprom_load(PITCH_PID_P_POS, pitch_pid_gains.p_gain);
  check_if_eeprom_loaded_nan(pitch_pid_gains.p_gain);
  eeprom_load(PITCH_PID_I_POS, pitch_pid_gains.i_gain);
  check_if_eeprom_loaded_nan(pitch_pid_gains.i_gain);
  eeprom_load(PITCH_PID_D_POS, pitch_pid_gains.d_gain);
  check_if_eeprom_loaded_nan(pitch_pid_gains.d_gain);
  eeprom_load(ROLL_PID_P_POS, roll_pid_gains.p_gain);
  check_if_eeprom_loaded_nan(roll_pid_gains.p_gain);
  eeprom_load(ROLL_PID_I_POS, roll_pid_gains.i_gain);
  check_if_eeprom_loaded_nan(roll_pid_gains.i_gain);
  eeprom_load(ROLL_PID_D_POS, roll_pid_gains.d_gain);
  check_if_eeprom_loaded_nan(roll_pid_gains.d_gain);
  eeprom_load(YAW_PID_P_POS, yaw_pid_gains.p_gain);
  check_if_eeprom_loaded_nan(yaw_pid_gains.p_gain);
  eeprom_load(YAW_PID_I_POS, yaw_pid_gains.i_gain);
  check_if_eeprom_loaded_nan(yaw_pid_gains.i_gain);
  eeprom_load(YAW_PID_D_POS, yaw_pid_gains.d_gain);
  check_if_eeprom_loaded_nan(yaw_pid_gains.d_gain);
  eeprom_load(PITCH_TRIM, pitch_trim);
  eeprom_load(ROLL_TRIM, roll_trim);
  eeprom_load(YAW_TRIM, yaw_trim);
}

void loop() {
  if (calibrationActive) {
    calibrateGimbals();
  } else if (millis() % 10 == 0) {  // Read gimbal values every 10ms
    set_gimbals();
    deadband();
    offset();
  }
  if (millis() % 80 == 0 && recieve_response(pkt)) {
    quadcopterArmed = pkt.armed;
  }
  check_arm_status();
  if (millis() % 50 == 0) {  // Send a packet every 50ms
    send_packet(throttle, yaw_offset, roll_offset, pitch_offset, quadcopterArmed,
      complementaryFilterGain, pitch_pid_gains, roll_pid_gains, yaw_pid_gains);
  }
  if (millis() % 1000 == 0) {
    lcd.clear();
  }
  if (millis() % 100 == 0) {
    updateLCD();
    if (calibrationActive) {
      print_range();
    } else {
      print_gimbals();
      // print_pid();
    }
  }
  if (quadcopterArmed) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void updateLCD() {
  lcd.setCursor(0, 0);
  if (quadcopterArmed) {
    lcd.setFastBacklight(255, 0, 0);  // red
  } else if (calibrationActive) {
    lcd.setFastBacklight(0, 255, 0);  // green
  } else {
    lcd.setFastBacklight(0, 0, 255);  // blue
  }
  if (calibrationActive) {
    lcd.print("Calibrating");
  } else if (tuningActive) {
    String msg = TUNING_PARAM_LABELS[currentTuningParamType];
    if (currentTuningParamType != COMPLEMENTARY_FILTER) {
      msg += TUNING_AXIS_LABELS[currentTuningAxis];
    }
    msg = msg + String(*currentTuningParam, 2);
    lcd.print(msg);
  } else if (trimmingActive) {
    lcd.print("Trim    Y: " + String(yaw_trim));
    lcd.setCursor(0, 1);
    String msg = "R: " + String(roll_trim) + "   P: " + String(pitch_trim);
    lcd.print(msg);
  } else {
    // Print some useful stats?
  }
}

void calibrateGimbals() {
  throttleRange[0] = min(throttleRange[0], analogRead(PIN_THROTTLE));
  throttleRange[1] = max(throttleRange[1], analogRead(PIN_THROTTLE));
  yawRange[0] = min(yawRange[0], analogRead(PIN_YAW));
  yawRange[1] = max(yawRange[1], analogRead(PIN_YAW));
  rollRange[0] = min(rollRange[0], analogRead(PIN_ROLL));
  rollRange[1] = max(rollRange[1], analogRead(PIN_ROLL));
  pitchRange[0] = min(pitchRange[0], analogRead(PIN_PITCH));
  pitchRange[1] = max(pitchRange[1], analogRead(PIN_PITCH));
}

void print_range() {
  Serial.print(F("Throttle: "));
  Serial.print(throttleRange[0]);
  Serial.print(F(" "));
  Serial.print(throttleRange[1]);

  Serial.print(F(" Yaw: "));
  Serial.print(yawRange[0]);
  Serial.print(F(" "));
  Serial.print(yawRange[1]);
  Serial.print(F(" "));
  Serial.print(yawRange[2]);

  Serial.print(F(" Roll: "));
  Serial.print(rollRange[0]);
  Serial.print(F(" "));
  Serial.print(rollRange[1]);
  Serial.print(F(" "));
  Serial.print(rollRange[2]);

  Serial.print(F(" Pitch: "));
  Serial.print(pitchRange[0]);
  Serial.print(F(" "));
  Serial.print(pitchRange[1]);
  Serial.print(F(" "));
  Serial.println(pitchRange[2]);
}

void print_pid() {
  Serial.print(F("comp_filter: "));
  Serial.print(complementaryFilterGain);
  Serial.print(F(" "));

  Serial.print(F("pitch: "));
  Serial.print(pitch_pid_gains.p_gain);
  Serial.print(F(" "));
  Serial.print(pitch_pid_gains.i_gain);
  Serial.print(F(" "));
  Serial.print(pitch_pid_gains.d_gain);
  Serial.print(F(" "));

  Serial.print(F("roll: "));
  Serial.print(roll_pid_gains.p_gain);
  Serial.print(F(" "));
  Serial.print(roll_pid_gains.i_gain);
  Serial.print(F(" "));
  Serial.print(roll_pid_gains.d_gain);
  Serial.print(F(" "));

  Serial.print(F("yaw: "));
  Serial.print(yaw_pid_gains.p_gain);
  Serial.print(F(" "));
  Serial.print(yaw_pid_gains.i_gain);
  Serial.print(F(" "));
  Serial.print(yaw_pid_gains.d_gain);
  Serial.println(F(""));
}

void set_gimbals() {
  throttle = analogRead(PIN_THROTTLE);
  throttle = map(throttle, throttleRange[0], throttleRange[1], AXIS_MIN, 200);
  yaw = analogRead(PIN_YAW);
  yaw = map(yaw, yawRange[0], yawRange[1], AXIS_MAX, AXIS_MIN);
  yaw = constrain(yaw + int((128 - yawRange[2] + yaw_trim) * offset_factor(yaw)), AXIS_MIN, AXIS_MAX);
  roll = analogRead(PIN_ROLL);
  roll = map(roll, rollRange[0], rollRange[1], AXIS_MIN, AXIS_MAX);
  roll = constrain(roll + int((128 - rollRange[2] + roll_trim) * offset_factor(roll)), AXIS_MIN, AXIS_MAX);
  pitch = analogRead(PIN_PITCH);
  pitch = map(pitch, pitchRange[0], pitchRange[1], AXIS_MAX, AXIS_MIN);
  pitch = constrain(pitch + int((128 - pitchRange[2] + pitch_trim) * offset_factor(pitch)), AXIS_MIN, AXIS_MAX);
}

float offset_factor(int raw_input) {
  return (1.0 - abs((raw_input - 128.0)/128.0));
}

void print_gimbals() {
  if (quadcopterArmed) {
    Serial.print("A ");
  } else {
    Serial.print(". ");
  }
  Serial.print(throttle);
  Serial.print("   ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(yaw_offset);
  Serial.print("   ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(roll_offset);
  Serial.print("   ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(pitch_offset);
}

void btn1_pressed(bool down) {
  if (down && !quadcopterArmed && !tuningActive && !calibrationActive && !trimmingActive) {
    calibrationActive = true;

    int curThrottle = analogRead(PIN_THROTTLE);
    int curYaw = analogRead(PIN_YAW);
    int curRoll = analogRead(PIN_ROLL);
    int curPitch = analogRead(PIN_PITCH);

    // Set default Min range
    throttleRange[0] = curThrottle;
    yawRange[0] = curYaw;
    rollRange[0] = curRoll;
    pitchRange[0] = curPitch;

    // Set default Max range
    throttleRange[1] = curThrottle;
    yawRange[1] = curYaw;
    rollRange[1] = curRoll;
    pitchRange[1] = curPitch;
  } else if (down && quadcopterArmed && !calibrationActive && !tuningActive && !trimmingActive) {
    trimmingActive = true;
  }
}

void btn2_pressed(bool down) {
  if (down && quadcopterArmed) {
    quadcopterArmed = false;
  }
  if (down && calibrationActive) {
    // Update the centers
    yawRange[2] = map(analogRead(PIN_YAW), yawRange[0], yawRange[1], AXIS_MAX, AXIS_MIN);
    rollRange[2] = map(analogRead(PIN_ROLL), rollRange[0], rollRange[1], AXIS_MIN, AXIS_MAX);
    pitchRange[2] = map(analogRead(PIN_PITCH), pitchRange[0], pitchRange[1], AXIS_MAX, AXIS_MIN);

    // Save
    eeprom_store(THR_POS, throttleRange);
    eeprom_store(YAW_POS, yawRange);
    eeprom_store(ROLL_POS, rollRange);
    eeprom_store(PIT_POS, pitchRange);

    calibrationActive = false;
  }
}

void check_arm_status() {
  if (!quadcopterArmed && !calibrationActive && throttle == 0 && yaw <= 5 && roll >= 250 &&
      pitch <= 5) {
    quadcopterArmed = true;
  }
}

void knob_pressed(bool down) {
  if (down){
    if (tuningActive) {
      eeprom_store(COMP_FILTER_POS, complementaryFilterGain);
      eeprom_store(PITCH_PID_P_POS, pitch_pid_gains.p_gain);
      eeprom_store(PITCH_PID_I_POS, pitch_pid_gains.i_gain);
      eeprom_store(PITCH_PID_D_POS, pitch_pid_gains.d_gain);
      eeprom_store(ROLL_PID_P_POS, roll_pid_gains.p_gain);
      eeprom_store(ROLL_PID_I_POS, roll_pid_gains.i_gain);
      eeprom_store(ROLL_PID_D_POS, roll_pid_gains.d_gain);
      eeprom_store(YAW_PID_P_POS, yaw_pid_gains.p_gain);
      eeprom_store(YAW_PID_I_POS, yaw_pid_gains.i_gain);
      eeprom_store(YAW_PID_D_POS, yaw_pid_gains.d_gain);

      tuningActive = false;
      currentTuningParam = NULL;
    } else if (trimmingActive) {
      eeprom_store(PITCH_TRIM, pitch_trim);
      eeprom_store(ROLL_TRIM, roll_trim);
      eeprom_store(YAW_TRIM, yaw_trim);

      trimmingActive = false;
    }
    knob1.setCurrentPos(0);
  }
}

void btn_down_pressed(bool down) {
  if (down && !calibrationActive && !trimmingActive) {
    currentTuningParamType = COMPLEMENTARY_FILTER;
    begin_tuning();
  } else if (down && !calibrationActive && !tuningActive && trimmingActive) {
    pitch_trim = constrain(pitch_trim - 1, -128, 127);
  }
}

void btn_left_pressed(bool down) {
  if (down && !calibrationActive && !trimmingActive) {
    currentTuningParamType = PROPORTIONAL;
    begin_tuning();
  } else if (down && !calibrationActive && !tuningActive && trimmingActive) {
    roll_trim = constrain(roll_trim - 1, -128, 127);
  }
}

void btn_up_pressed(bool down) {
  if (down && !calibrationActive && !trimmingActive) {
    currentTuningParamType = INTEGRAL;
    begin_tuning();
  } else if (down && !calibrationActive && !tuningActive && trimmingActive) {
    pitch_trim = constrain(pitch_trim + 1, -128, 127);
  }
}

void btn_right_pressed(bool down) {
  if (down && !calibrationActive && !trimmingActive) {
    currentTuningParamType = DERIVATIVE;
    begin_tuning();
  } else if (down && !calibrationActive && !tuningActive && trimmingActive) {
    roll_trim = constrain(roll_trim + 1, -128, 127);
  }
}

void btn_center_pressed(bool down) {
  if (down && tuningActive && !calibrationActive && !trimmingActive) {
    if (currentTuningAxis == PITCH) {
      currentTuningAxis = ROLL;
    } else if (currentTuningAxis == ROLL) {
      currentTuningAxis = YAW;
    } else if (currentTuningAxis == YAW) {
      currentTuningAxis = PITCH;
    }
    Serial.println(TUNING_AXIS_LABELS[currentTuningAxis]);
    begin_tuning();
  }
}

void begin_tuning() {
  if (calibrationActive || trimmingActive) {
    return;
  }
  tuningActive = true;
  pid_gains* gains = NULL;
  switch (currentTuningAxis) {
    case PITCH:
      gains = &pitch_pid_gains;
      break;
    case ROLL:
      gains = &roll_pid_gains;
      break;
    case YAW:
      gains = &yaw_pid_gains;
      break;
  }

  switch (currentTuningParamType) {
    case COMPLEMENTARY_FILTER:
      currentTuningParam = &complementaryFilterGain;
      break;
    case PROPORTIONAL:
      currentTuningParam = &(gains->p_gain);
      break;
    case INTEGRAL:
      currentTuningParam = &(gains->i_gain);
      break;
    case DERIVATIVE:
      currentTuningParam = &(gains->d_gain);
      break;
  }
  knob1.setCurrentPos(0);
}

void knobs_update() {
  if (tuningActive) {
    float knob_divider = 100.0;
    float gain_min = 0.0;
    float gain_max = 1.0;
    
    switch (currentTuningParamType) {
      case COMPLEMENTARY_FILTER:
        break;
      case PROPORTIONAL:
      case INTEGRAL:
      case DERIVATIVE:
        knob_divider = 20.0;
        gain_max = 2.5;
        break;
      default:
        break;
    }

    *currentTuningParam = constrain(*currentTuningParam + (knob1.getCurrentPos() / knob_divider), gain_min, gain_max);
  } else if (trimmingActive) {
    yaw_trim = constrain(yaw_trim + knob1.getCurrentPos(), -128, 127);
  }
  knob1.setCurrentPos(0);
}

void check_if_eeprom_loaded_nan(float& param) {
  if (isnan(param)) {
    Serial.println("Error loading param from EEPROM, setting to 0.0");
    param = 0.0;
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

void offset() {
  const int YAW_OFFSET_ANGLE_MAX = 125;  // degrees
  const int TILT_OFFSET_ANGLE_MAX = 10;  // degrees

  pitch_offset = map(pitch, 0, 255, -TILT_OFFSET_ANGLE_MAX, TILT_OFFSET_ANGLE_MAX);
  roll_offset = map(roll, 0, 255, -TILT_OFFSET_ANGLE_MAX, TILT_OFFSET_ANGLE_MAX);
  yaw_offset = map(yaw, 0, 255, -YAW_OFFSET_ANGLE_MAX, YAW_OFFSET_ANGLE_MAX);
}
