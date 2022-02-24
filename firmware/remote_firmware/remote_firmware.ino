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

const int YAW_TRIM = 88;

const int AXIS_MIN = 0;
const int AXIS_MAX = 255;

const int SERIAL_BAUD = 9600;  // Baud rate for serial port

enum TuningParam {
  NONE,
  COMPLEMENTARY_FILTER,
  PROPORTIONAL,
  INTEGRAL,
  DERIVATIVE,
  TRIM
};

enum TuningAxis {
  PITCH,
  ROLL,
  YAW
};

const char* TUNING_AXIS_LABELS[] = {"Pitch", "Roll", "Yaw"};
const char* TUNING_PARAM_LABELS[] = {"", "Filter Gain:", "P", "I", "D", "Trim"};


// Global Variables
int throttleRange[3];
int yawRange[3];
int rollRange[3];
int pitchRange[3];

response_pkt pkt;

bool calibrationActive = false;
bool quadcopterArmed = false;

bool tuningActive = false;
TuningParam currentTuningParamType = NONE;
float* currentTuningParam = NULL;
TuningAxis currentTuningAxis = PITCH;

int yaw = 0;
int throttle = 0;
int roll = 0;
int pitch = 0;

float complementaryFilterGain = 0.98;
pid_gains pitch_pid_gains;
pid_gains roll_pid_gains;
pid_gains yaw_pid_gains;
float yaw_trim = 0;

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
void display_tuning_param();
void begin_tuning();
void check_if_eeprom_loaded_nan(float&);


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
  lcd.setBacklight(0x000000FF);

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
  eeprom_load(YAW_TRIM, yaw_trim);
  check_if_eeprom_loaded_nan(yaw_trim);
}

void loop() {
  if (calibrationActive) {
    calibrateGimbals();
  } else {
    if (millis() % 10 == 0) {  // Read gimbal values every 10ms
      set_gimbals();
    }
    if (millis() % 80 == 0) {
      if (recieve_response(pkt)) {
        quadcopterArmed = pkt.armed;
      }
    }
    check_arm_status();
    if (millis() % 50 == 0) {  // Send a packet every 50ms
      send_packet(throttle, yaw, roll, pitch, quadcopterArmed,
        complementaryFilterGain, pitch_pid_gains, roll_pid_gains, yaw_pid_gains);
    }
    if (millis() % 100 == 0 && tuningActive) {
      display_tuning_param();
    }
  }
  if (millis() % 100 == 0) {
    print_gimbals();
    //print_pid();
  }
  if (quadcopterArmed) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void calibrateGimbals() {
  print_range();

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
  delay(500);

  while (calibrationActive) {
    // Get new gimbal values
    curThrottle = analogRead(PIN_THROTTLE);
    curYaw = analogRead(PIN_YAW);
    curRoll = analogRead(PIN_ROLL);
    curPitch = analogRead(PIN_PITCH);

    // Update range if need be
    if (curThrottle < throttleRange[0]) {
      throttleRange[0] = curThrottle;
    }
    if (curThrottle > throttleRange[1]) {
      throttleRange[1] = curThrottle;
    }

    if (curYaw < yawRange[0]) {
      yawRange[0] = curYaw;
    }
    if (curYaw > yawRange[1]) {
      yawRange[1] = curYaw;
    }

    if (curRoll < rollRange[0]) {
      rollRange[0] = curRoll;
    }
    if (curRoll > rollRange[1]) {
      rollRange[1] = curRoll;
    }

    if (curPitch < pitchRange[0]) {
      pitchRange[0] = curPitch;
    }
    if (curPitch > pitchRange[1]) {
      pitchRange[1] = curPitch;
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Getting center");
  delay(1500);
  //last read gets the new center
  curYaw = analogRead(PIN_YAW);
  curRoll = analogRead(PIN_ROLL);
  curPitch = analogRead(PIN_PITCH);

  curYaw = map(curYaw, yawRange[0], yawRange[1], AXIS_MAX, AXIS_MIN);
  curRoll = map(curRoll, rollRange[0], rollRange[1], AXIS_MIN, AXIS_MAX);
  curPitch = map(curPitch, pitchRange[0], pitchRange[1], AXIS_MAX, AXIS_MIN);

  yawRange[2] = curYaw;
  rollRange[2] = curRoll;
  pitchRange[2] = curPitch;

  eeprom_store(THR_POS, throttleRange);
  eeprom_store(YAW_POS, yawRange);
  eeprom_store(ROLL_POS, rollRange);
  eeprom_store(PIT_POS, pitchRange);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Closing Calibration");
  delay(1000);
  lcd.clear();

  // Debug
  print_range();
  return;
}

void print_range() {
  Serial.print("\nThrottle Min ");
  Serial.print(throttleRange[0]);
  Serial.print("\nThrottle Max ");
  Serial.print(throttleRange[1]);

  Serial.print("\nYaw Min ");
  Serial.print(yawRange[0]);
  Serial.print("\nYaw Max ");
  Serial.print(yawRange[1]);
  Serial.print("\nYaw Center ");
  Serial.print(yawRange[2]);

  Serial.print("\nRoll Min ");
  Serial.print(rollRange[0]);
  Serial.print("\nRoll Max ");
  Serial.print(rollRange[1]);
  Serial.print("\nRoll Center ");
  Serial.print(rollRange[2]);

  Serial.print("\nPitch Min ");
  Serial.print(pitchRange[0]);
  Serial.print("\nPitch Max ");
  Serial.print(pitchRange[1]);
  Serial.print("\nPitch Center ");
  Serial.print(pitchRange[2]);
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
  throttle =
      map(throttle, throttleRange[0], throttleRange[1], AXIS_MIN, 200);
  if (throttle <= 15) {
    throttle = 0;
  }
  yaw = analogRead(PIN_YAW);
  yaw = map(yaw, yawRange[0], yawRange[1], AXIS_MAX, AXIS_MIN);
  yaw = yaw + int((128 - yawRange[2] - (int)yaw_trim) * offset_factor(yaw));
  roll = analogRead(PIN_ROLL);
  roll = map(roll, rollRange[0], rollRange[1], AXIS_MIN, AXIS_MAX);
  roll = roll + int((128 - rollRange[2]) * offset_factor(roll));
  pitch = analogRead(PIN_PITCH);
  pitch = map(pitch, pitchRange[0], pitchRange[1], AXIS_MAX, AXIS_MIN);
  pitch = pitch + int((128 - pitchRange[2]) * offset_factor(pitch));
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
  Serial.print("  ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(yawRange[2]);
  Serial.print("  ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(rollRange[2]);
  Serial.print("  ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(pitchRange[2]);
}

void btn1_pressed(bool down) {
  if (down && !quadcopterArmed && !tuningActive && !calibrationActive) {
    calibrationActive = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Calibrating");
  }
  else if (down && quadcopterArmed && !calibrationActive){
    currentTuningParamType = TRIM;
    currentTuningAxis = YAW;
    begin_tuning();
  }
}

void btn2_pressed(bool down) {
  if (down && quadcopterArmed) {
    quadcopterArmed = false;
    lcd.setBacklight(0x000000FF);
  }
  if (down && calibrationActive) {
    calibrationActive = false;
  }
}

void check_arm_status() {
  if (!quadcopterArmed && throttle == 0 && yaw <= 5 && roll >= 250 &&
      pitch <= 5) {
    quadcopterArmed = true;
    lcd.setBacklight(0x00FF0000);
  }
}

void knob_pressed(bool down) {
  if (down){
    if(tuningActive) {
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
      eeprom_store(YAW_TRIM, yaw_trim);

      tuningActive = false;
      currentTuningParam = NULL;

      lcd.clear();
      knob1.setCurrentPos(0);
    }
  }
}

void btn_down_pressed(bool down) {
  if (down && !calibrationActive) {
    currentTuningParamType = COMPLEMENTARY_FILTER;
    begin_tuning();
  }
}

void btn_left_pressed(bool down) {
  if (down && !calibrationActive) {
    currentTuningParamType = PROPORTIONAL;
    begin_tuning();
  }
}

void btn_up_pressed(bool down) {
  if (down && !calibrationActive) {
    currentTuningParamType = INTEGRAL;
    begin_tuning();
  }
}

void btn_right_pressed(bool down) {
  if (down && !calibrationActive) {
    currentTuningParamType = DERIVATIVE;
    begin_tuning();
  }
}

void btn_center_pressed(bool down) {
  if (down && tuningActive && !calibrationActive) {
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
    case TRIM:
      currentTuningParam = &yaw_trim;
      break;
  }

  lcd.clear();
  lcd.setCursor(0, 0);

  String msg = TUNING_AXIS_LABELS[currentTuningAxis];
  if (currentTuningParamType != COMPLEMENTARY_FILTER && currentTuningParamType != TRIM) {
    msg = msg + " " + TUNING_PARAM_LABELS[currentTuningParamType];
  }

  lcd.print(msg);
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
      case TRIM:
        knob_divider = 1;
        gain_min = -128;
        gain_max = 127;
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
    knob1.setCurrentPos(0);
  }
}

void display_tuning_param() {
  lcd.setCursor(0, 1);
  lcd.print(*currentTuningParam, 2);
}

void check_if_eeprom_loaded_nan(float& param) {
  if (isnan(param)) {
    Serial.println("Error loading param from EEPROM, setting to 0.0");
    param = 0.0;
  }
}
